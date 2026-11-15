// Uses the already deployed Even SDK. This page never changes glasses firmware.
export const POLL_MS = 150;
export const POLL_BACKOFF_MS = 350;
export const HTTP_TIMEOUT_MS = 1500;
export const STALE_MS = 900;
const MODE_NAMES = {STOP:'停止', MANUAL:'手動', STEER_AUTO:'操舵自動', AUTO:'自動走行'};
const CAPTURE_ID = 1;
const CAPTURE_NAME = 'magp-hud';
const EMERGENCY_RECOVERY = {release_stop_buttons:'停止ボタンを離してください',release_controls:'R2・L2とスティック・十字キーを離す',press_cross:'×で解除・手動Nへ復帰',controller_disconnected:'コントローラーを接続してください',input_stale:'コントローラー入力の復旧待ち'};
const REJECTIONS = {emergency_button:'このRCは操作入力を離し、×で解除・手動Nへ復帰してください。',emergency_stop:'このRCは操作入力を離し、×で解除・手動Nへ復帰してください。',controller_disconnected:'コントローラーが未接続です。',controls_not_released:'R2・L2を離してから操作してください。',unsupported_mode:'この走行モードは選択できません。'};
export function emergencyHint(state) {return ['emergency_stop','emergency_button'].includes(state?.reason)?(Object.hasOwn(EMERGENCY_RECOVERY,state.emergency_recovery)?EMERGENCY_RECOVERY[state.emergency_recovery]:'停止ボタンと操作入力を離し、×で解除'):null;}

export function validStatus(value) {
  return value && Number.isSafeInteger(value.seq) && value.seq >= 0
    && Number.isFinite(value.server_time_ms) && Object.hasOwn(MODE_NAMES, value.mode)
    && ['D','N','R'].includes(value.gear) && typeof value.enabled === 'boolean'
    && typeof value.reason_text === 'string' && value.reason_text.length <= 1000
    && value.controller && typeof value.controller.connected === 'boolean'
    && value.perception && value.speed && typeof value.speed.valid === 'boolean'
    && (!value.speed.valid || (typeof value.speed.kph === 'number' && Number.isFinite(value.speed.kph)))
    && value.output && (value.last_request_id === null || typeof value.last_request_id === 'string');
}

export class StatusMonitor {
  constructor({fetcher=globalThis.fetch.bind(globalThis), now=()=>performance.now(), onChange=()=>{}, setTimer=(fn,ms)=>globalThis.setTimeout(fn,ms), clearTimer=id=>globalThis.clearTimeout(id)}={}) {
    Object.assign(this,{fetcher,now,onChange,setTimer,clearTimer});
    this.status=null; this.receivedAt=-Infinity; this.progressAt=-Infinity;
    this.pending=null; this.feedback=''; this.closed=false; this.active=false; this.invalidated=false;
    this.pollTimer=null; this.pollWake=null; this.pollEpoch=0; this.requestAbort=null; this.modeAbort=null; this.polling=false; this.readTask=null; this.readEpoch=0; this.intentEpoch=0; this.preflighting=false;
  }
  fresh() { return !this.closed && !this.invalidated && !!this.status && this.now()-this.receivedAt < STALE_MS && this.now()-this.progressAt < STALE_MS; }
  ageMs() { return this.status ? Math.max(0,Math.round(Math.max(this.now()-this.receivedAt,this.now()-this.progressAt))) : null; }
  accept(value) {
    if (!validStatus(value)) throw new Error('車両の状態形式を確認できません');
    const restarted=this.status && (value.seq < this.status.seq || value.server_time_ms < this.status.server_time_ms);
    if (!this.status || restarted || value.seq !== this.status.seq) this.progressAt=this.now();
    if (restarted && this.pending) { this.pending=null; this.feedback='車両が再接続されました。切替は再送していません。'; }
    this.status=value; this.receivedAt=this.now(); this.invalidated=false;
    if (this.pending && value.last_request_id === this.pending.id) {
      this.feedback=value.mode === this.pending.mode
        ? `${MODE_NAMES[value.mode]}への切替を車両で確認しました。`
        : `切替されませんでした：${value.reason_text || MODE_NAMES[value.mode]}`;
      this.pending=null;
    }
    this.checkPending(); this.onChange();
  }
  checkPending() {
    if (this.pending && this.now()-this.pending.at > 2500) {
      this.pending=null; this.feedback='切替結果が未確認です。現在の状態を確認してください。要求は再送していません。';
    }
  }
  readOnce({paced=false}={}) {
    if(this.closed)return Promise.resolve(false);
    if(this.readTask)return this.readTask;
    this.polling=true;
    const epoch=this.readEpoch;
    const abort=new AbortController(); this.requestAbort=abort;
    const operation=Promise.resolve().then(async()=>{
      if(this.closed||epoch!==this.readEpoch)return false;
      const timeout=this.setTimer(()=>abort.abort(),HTTP_TIMEOUT_MS);
      try {
        const response=await this.fetcher(paced?'/hud/status/wait':'/hud/status.json',{cache:'no-store',credentials:'same-origin',signal:abort.signal,headers:{Accept:'application/json'}});
        if(!response.ok)throw new Error('状態を受信できません');
        const value=await response.json();
        if(this.closed||epoch!==this.readEpoch)return false;
        this.accept(value);return true;
      }catch(_){if(!this.closed&&epoch===this.readEpoch){this.checkPending();this.onChange();}return false;}
      finally{this.clearTimer(timeout);}
    });
    this.readTask=operation.finally(()=>{this.polling=false;if(this.requestAbort===abort)this.requestAbort=null;this.readTask=null;});
    return this.readTask;
  }
  start() {
    if(this.closed || this.active)return; this.active=true;
    const epoch=++this.pollEpoch;
    const current=()=>this.active&&!this.closed&&epoch===this.pollEpoch;
    const poll=async()=>{
      while(current()){
        // The server paces successful reads. Network completion is not subject
        // to iOS's background setTimeout floor, unlike a client polling timer.
        const ok=await this.readOnce({paced:true});
        if(!current())return;
        if(!ok)await new Promise(resolve=>{this.pollWake=resolve;this.pollTimer=this.setTimer(()=>{this.pollTimer=null;this.pollWake=null;resolve();},POLL_BACKOFF_MS);});
      }
    };
    void poll();
  }
  suspend() { this.active=false;this.pollEpoch++;this.readEpoch++;this.intentEpoch++;this.clearTimer(this.pollTimer);this.pollTimer=null;this.pollWake?.();this.pollWake=null;this.requestAbort?.abort();this.invalidated=true;this.onChange(); }
  cancelUnsentToggle(){this.intentEpoch++;if(this.preflighting&&!this.pending&&!this.closed){this.feedback='切替前の確認を中断しました。もう一度タップしてください。';this.onChange();}}
  async toggle() {
    if(this.closed||this.pending||this.preflighting)return false;
    this.preflighting=true;
    const epoch=this.readEpoch,intent=this.intentEpoch;let abort=null,timeout=null,id=null;
    try {
      if(!this.fresh()){
        this.feedback='切替前に新しい車両状態を確認中…';this.onChange();
        await this.readOnce();
      }
      if(this.closed||epoch!==this.readEpoch||intent!==this.intentEpoch)return false;
      if(!this.fresh()){this.feedback='通信状態が古いため切替できません。新しい状態を待ってください。';this.onChange();return false;}
      const mode=this.status.mode==='AUTO'?'MANUAL':'AUTO';
      id=globalThis.crypto?.randomUUID?.() ?? `g2-${Date.now()}-${Math.random().toString(16).slice(2)}`;
      this.pending={id,mode,at:this.now()};this.feedback=`${MODE_NAMES[mode]}へ切替要求中…`;this.onChange();
      abort=new AbortController();this.modeAbort=abort;
      timeout=this.setTimer(()=>abort.abort(),HTTP_TIMEOUT_MS);
      const response=await this.fetcher('/hud/mode',{method:'POST',cache:'no-store',credentials:'same-origin',headers:{'Content-Type':'application/json',Accept:'application/json'},body:JSON.stringify({mode,request_id:id}),signal:abort.signal});
      if(!response.ok)throw new Error('要求を受け付けられません');
      const reply=await response.json();
      if(reply.ok===false){
        if(this.pending?.id===id){this.pending=null;const rejectedForEmergency=['emergency_button','emergency_stop'].includes(reply.reason);let instruction=rejectedForEmergency?(emergencyHint(this.status)??REJECTIONS[reply.reason]):REJECTIONS[reply.reason];if(rejectedForEmergency&&!instruction.includes('×'))instruction+='。このRCは×で解除・手動Nへ復帰します。';this.feedback=`切替されませんでした：${instruction??'車両側で要求が拒否されました。'}`;this.onChange();}
        return false;
      }
      if(reply.ok!==true)throw new Error('受付を確認できません');
      // A queued response is not an applied response. Only accept() confirms it.
      if(this.pending?.id===id){this.feedback='受付済み・車両の切替結果を確認中…';this.onChange();}
      return true;
    } catch (_) {
      if(this.pending?.id===id){this.pending=null;this.feedback='受付が未確認です。要求は再送していません。現在の状態を確認してください。';this.onChange();}
      return false;
    } finally {if(timeout!==null)this.clearTimer(timeout);if(this.modeAbort===abort)this.modeAbort=null;this.preflighting=false;}
  }
  close(){this.closed=true;this.suspend();this.modeAbort?.abort();}
}

// SDK 0 may omit eventType for CLICK. Do not turn any other raw event into a tap.
function rawClickOnly(raw,depth=0) {
  if(!raw||typeof raw!=='object'||Array.isArray(raw))return true;
  if(depth>4)return false;
  return Object.entries(raw).every(([key,value])=>{
    const normalized=key.toLowerCase().replace(/[^a-z0-9]/g,'');
    if(normalized==='eventtype')return value===0||value==='0'||value==='CLICK'||value==='CLICK_EVENT';
    return ['data','jsondata','textevent','sysevent','listevent'].includes(normalized)?rawClickOnly(value,depth+1):true;
  });
}
export function nativeEventType(event) {
  const text=event?.textEvent;
  const capture=text?.containerID===CAPTURE_ID && (text.containerName===undefined||text.containerName===CAPTURE_NAME);
  const sys=event?.sysEvent;
  const source=!!sys && [1,2,3].includes(sys.eventSource??0);
  if(!capture&&!source)return null;
  const type=capture?text.eventType:sys.eventType;
  return type===undefined&&rawClickOnly(event.jsonData)?0:Number.isInteger(type)?type:null;
}

export class NativeDisplay {
  constructor({waitForBridge,onTap=()=>{},onDetail=()=>{},onState=()=>{},onOverlayChange=()=>{},now=()=>performance.now()}={}) {
    Object.assign(this,{waitForBridge,onTap,onDetail,onState,onOverlayChange,now});
    this.bridge=null;this.connected=false;this.foreground=true;this.suspended=false;this.closed=false;this.failed=false;
    this.pending=null;this.startupPending=null;this.sending=false;this.sent=new Map();this.pressHeld=false;this.releaseFence=false;this.lastTap=-Infinity;
  }
  async connect(initial=[]) {
    if(this.connected||this.closed||this.failed)return;
    try{
      this.bridge=await this.waitForBridge();if(this.closed)return;
      if(this.suspended){this.startupPending=initial;return;}
      this.unsubscribe=this.bridge.onEvenHubEvent(event=>this.onEvent(event));
      const capture={containerID:CAPTURE_ID,containerName:CAPTURE_NAME,xPosition:0,yPosition:0,width:576,height:288,borderWidth:0,paddingLength:0,isEventCapture:1,content:' ',zOrderIndex:0};
      const textObject=[capture,...initial.map((item,index)=>({...item,borderWidth:0,paddingLength:0,isEventCapture:0,zOrderIndex:index+1}))];
      const result=await this.nativeCall(()=>this.bridge.createStartUpPageContainer({containerTotalNum:textObject.length,textObject}));
      if(result!==0)throw new Error('G2画面を作成できません');
      if(this.closed)return;this.connected=true;
      for(const field of initial)this.sent.set(field.containerID,field.content);
      this.onState('G2へ表示中');if(this.pending)void this.flush();
    }catch(_){this.fail();}
  }
  nativeCall(call){return new Promise((resolve,reject)=>{const timer=setTimeout(()=>reject(new Error('G2応答待ち')),5000);Promise.resolve().then(call).then(value=>{clearTimeout(timer);resolve(value);},error=>{clearTimeout(timer);reject(error);});});}
  update(fields){this.pending=fields;if(this.connected&&!this.sending&&this.foreground&&!this.suspended&&!this.failed&&!this.closed)void this.flush();}
  async flush(){
    if(this.sending||!this.connected||!this.foreground||this.suspended||this.failed||this.closed)return;
    this.sending=true;
    try{
      while(this.pending&&!this.closed&&this.foreground&&!this.suspended){
        const fields=this.pending;this.pending=null;
        for(const field of fields){
          if(this.closed||!this.foreground||this.suspended)break;
          if(this.sent.get(field.containerID)===field.content)continue;
          const ok=await this.nativeCall(()=>this.bridge.textContainerUpgrade({containerID:field.containerID,containerName:field.containerName,content:field.content}));
          if(ok!==true)throw new Error('G2表示の応答を確認できません');
          this.sent.set(field.containerID,field.content);
          if(this.pending)break; // replace obsolete fields with the newest view
        }
      }
    }catch(_){this.fail();}finally{this.sending=false;}
  }
  onEvent(event){
    if(this.closed)return;
    // Lifecycle may arrive without a touch source.
    const lifecycle=event?.sysEvent?.eventType;
    if(lifecycle===7||lifecycle===6){this.close();this.onState('G2表示終了');return;}
    // Even's foreground events describe the OS contextual-menu overlay:
    // 4 opens it; 5 closes it. The app and its network receiver remain alive.
    if(lifecycle===4){this.foreground=false;this.pressHeld=false;this.releaseFence=false;this.onOverlayChange(true);return;}
    if(lifecycle===5){this.foreground=true;this.pressHeld=false;this.releaseFence=false;this.onOverlayChange(false);if(this.pending)void this.flush();return;}
    if(!this.connected||!this.foreground||this.suspended||this.failed)return;
    const type=nativeEventType(event);
    if(type===3)return;
    if(type===9){if(!this.pressHeld){this.pressHeld=true;this.releaseFence=false;this.onDetail();}return;}
    if(type===10){if(this.pressHeld){this.pressHeld=false;this.releaseFence=true;queueMicrotask(()=>{this.releaseFence=false;});}return;}
    if(type===0&&!this.pressHeld&&!this.releaseFence&&this.now()-this.lastTap>=350){this.lastTap=this.now();this.onTap();}
  }
  fail(){this.failed=true;this.connected=false;this.pending=null;this.unsubscribe?.();this.onState('G2表示を確認できません。画面を開き直してください');}
  suspend(){this.suspended=true;this.pressHeld=false;this.releaseFence=false;}
  resume(){if(this.closed)return;this.suspended=false;if(this.startupPending){const fields=this.pending??this.startupPending;this.startupPending=null;void this.connect(fields);}else if(this.pending)void this.flush();}
  close(){this.closed=true;this.connected=false;this.pending=null;this.startupPending=null;this.unsubscribe?.();}
}

function clean(value,length=58){return String(value??'—').replace(/[\r\n\t]/g,' ').slice(0,length);}
function finiteText(value,suffix=''){return typeof value==='number'&&Number.isFinite(value)?`${Math.round(value)}${suffix}`:'—';}
function outputSettingText(value){return typeof value==='number'&&Number.isFinite(value)?`${value.toFixed(1)}%`:'—';}
export function hudFields(monitor,detail=false) {
  const state=monitor.status,fresh=monitor.fresh();
  const speed=state?.speed?.valid?Math.abs(state.speed.kph).toFixed(1):'—';
  const mode=state?MODE_NAMES[state.mode]:'状態待ち';
  const controller=state?.controller?.connected?'DualSense 接続':'DualSense 未接続';
  const recovery=emergencyHint(state);
  const reason=fresh?(recovery?`緊急停止中：${recovery}`:state.reason_text):state?'通信が古いため最終状態を表示':'車両との接続を待っています';
  const feedback=monitor.pending?`${MODE_NAMES[monitor.pending.mode]} 切替確認中`:monitor.feedback;
  const pace=state?.mode==='AUTO'&&state?.enabled&&state?.gear==='D'&&state?.corner_pace?.active?state.corner_pace:null;
  const paceNames={BASELINE:'通常走行',APPROACH:'カーブ減速',CORNER:'カーブ減速',RECOVERY:'白線再確認・減速',EXIT:'立上がり',STRAIGHT:'直線'};
  const paceDetail=pace?`${paceNames[pace.phase]??'—'} / 上限 ${outputSettingText(pace.cap_percent)} / 指令 ${outputSettingText(pace.applied_percent)}`:'カーブ出力上限 —';
  const texts=[
    ['magp-header',12,4,552,32,detail?`${fresh?'LIVE':'通信待ち'} / 自動出力設定 ${outputSettingText(state?.auto_output_setting_percent)}`:`MAGP / ${fresh?'LIVE':'通信待ち'}`],
    ['magp-mode',12,48,300,44,`${state&&!fresh?'最終 ':''}${mode} / ${state?.gear??'—'}`],
    ['magp-speed',326,48,238,44,`${speed} km/h ${state?.speed?.valid?'換算':'未計測'}`],
    ['magp-reason',12,104,552,64,clean(reason,54)],
    ['magp-controller',12,176,552,32,detail?paceDetail:controller],
    ['magp-feedback',12,226,552,46,clean(feedback||(detail?`指令値 / 白線 ${state?.perception?.mode??'—'}`:'タップ: 自動/手動　長押し: 詳細'),56)]
  ];
  return texts.map(([containerName,xPosition,yPosition,width,height,content],index)=>({containerID:index+2,containerName,xPosition,yPosition,width,height,content}));
}

export function boot(document=globalThis.document,{waitForBridge=async()=>{const sdk=await import('./index-Dl3TIl4p.js');return sdk.waitForEvenAppBridge();},eventTarget=globalThis}={}) {
  const element=id=>document.getElementById(id);let detail=false,native=null,nativeSessionStarted=false,pageSuspended=false,disposed=false;
  const setText=(id,text)=>{const node=element(id);if(node&&node.textContent!==text)node.textContent=text;};
  const monitor=new StatusMonitor({onChange:render});
  function render(){
    const state=monitor.status,fresh=monitor.fresh(),age=monitor.ageMs();monitor.checkPending();
    document.body.dataset.stale=String(!fresh);document.body.dataset.mode=state?.mode??'STOP';
    setText('link',fresh?'車両と接続中':state?'通信が古い / 最終状態':'車両へ接続中');
    element('link-dot').className=`dot ${fresh?'live':'stale'}`;
    setText('age',age===null?'—':age<1000?`${age} ms前`:`${(age/1000).toFixed(1)}秒前`);
    setText('state-caption',fresh?'車両で確認した現在の状態':state?'最終受信時の状態':'車両からの状態を待っています');
    setText('mode',state?MODE_NAMES[state.mode]:'—');setText('gear',state?.gear??'—');
    setText('speed',state?.speed?.valid?Math.abs(state.speed.kph).toFixed(1):'—');
    setText('speed-note',state?.speed?.valid?'設定値で換算・未校正':'未計測（新鮮な回転信号なし）');
    setText('auto-setting',outputSettingText(state?.auto_output_setting_percent));
    const recovery=emergencyHint(state);
    setText('reason',fresh?(recovery?`緊急停止中：${recovery}`:state.reason_text):state?'通信が途切れています。以下の表示は最終受信時の状態です。':'状態を取得しています');
    setText('controller',state?.controller?.connected?`${clean(state.controller.device||'DualSense')} / ${finiteText(state.controller.input_age_ms,' ms')}`:'未接続');
    setText('perception',state?`${state.perception.mode??'—'} / ${typeof state.perception.confidence==='number'?Math.round(state.perception.confidence*100)+'%':'—'}`:'—');
    setText('steering',finiteText(state?.output?.steer_us,' µs'));setText('throttle',finiteText(state?.output?.throttle_us,' µs'));
    setText('forward',finiteText(state?.output?.forward_percent,' %'));setText('seq',state?String(state.seq):'—');
    const button=element('toggle');button.disabled=!fresh||!!monitor.pending;
    setText('toggle',monitor.pending?'切替結果を確認中…':state?.mode==='AUTO'?'手動へ切り替える':'自動走行へ切り替える');
    setText('feedback',monitor.feedback||'G2を1回タップすると自動／手動を切り替えます。');
    native?.update(hudFields(monitor,detail));
  }
  async function connectGlasses(){
    const button=element('connect-glasses');if(button.disabled)return;button.disabled=true;setText('glasses','Evenアプリとの接続待ち');
    native=new NativeDisplay({waitForBridge,onTap:()=>void monitor.toggle(),onDetail:()=>{detail=!detail;render();},onOverlayChange:open=>{if(open)monitor.cancelUnsentToggle();if(!pageSuspended)monitor.start();},onState:message=>{setText('glasses',message);if(native?.connected)nativeSessionStarted=true;if(native?.closed||native?.failed)monitor.cancelUnsentToggle();syncVisibility();}});
    if(pageSuspended)native.suspend();
    await native.connect(hudFields(monitor,detail));
    if(native.failed)button.disabled=false;
  }
  element('toggle').addEventListener('click',()=>void monitor.toggle());
  element('connect-glasses').addEventListener('click',()=>void connectGlasses());
  function syncVisibility(){
    // Even keeps the iOS WebView alive behind the glasses UI. Hiding that
    // WebView does not mean the native glasses page or its receiver has ended.
    if(pageSuspended)monitor.suspend();
    else if(!document.hidden||nativeSessionStarted)monitor.start();
    else monitor.suspend();
  }
  document.addEventListener('visibilitychange',syncVisibility);
  const repaint=setInterval(render,150);monitor.start();render();
  function close(){if(disposed)return;disposed=true;clearInterval(repaint);native?.suspend();monitor.close();native?.close();eventTarget.removeEventListener?.('pagehide',pageHide);eventTarget.removeEventListener?.('pageshow',pageShow);}
  function pageHide(event){if(event.persisted){pageSuspended=true;native?.suspend();monitor.suspend();}else close();}
  function pageShow(event){if(event.persisted&&!disposed){pageSuspended=false;monitor.start();native?.resume();}}
  eventTarget.addEventListener?.('pagehide',pageHide);
  eventTarget.addEventListener?.('pageshow',pageShow);
  // The SDK is safe to await in a normal browser, but no automatic reconnect loop.
  void connectGlasses();
  return {monitor,get display(){return native;},close};
}
if(typeof document!=='undefined'&&document.getElementById('mode'))boot(document);
