const $=id=>document.getElementById(id);let device,server,char,pts=[],pos={x:0,y:0,z:0},vel={x:0,y:0,z:0},last=0,running=false,lastStill=0,rx=[],live={ax:0,ay:0,az:0,roll:0,pitch:0,yaw:0};
const SERVICE_FILTERS=[]; // connect by device name; discover characteristics after pairing
$('connect').onclick=async()=>{try{device=await navigator.bluetooth.requestDevice({filters:[{namePrefix:'WT'}],optionalServices:['0000ffe5-0000-1000-8000-00805f9a34fb']});server=await device.gatt.connect();let services=await server.getPrimaryServices();let candidates=[];for(const s of services){let cs=await s.getCharacteristics();for(const c of cs)if(c.properties.notify||c.properties.indicate)candidates.push(c)}if(!candidates.length)throw Error('No notification characteristic found');char=candidates.find(c=>c.uuid.toLowerCase().includes('ffe4'))||candidates.find(c=>c.uuid.toLowerCase().includes('ffe1'))||candidates[0];await char.startNotifications();char.addEventListener('characteristicvaluechanged',packet);$('status').textContent='Connected • '+char.uuid.slice(4,8);$('status').classList.add('on');}catch(e){alert('Bluetooth connection failed: '+e.message)}};
function packet(e){
  const incoming=new Uint8Array(e.target.value.buffer,e.target.value.byteOffset,e.target.value.byteLength);
  const liveEl=document.getElementById('liveSensor');
  // WT9011DCL-BT50 BLE default packet is exactly 20 bytes:
  // 55 61 + accel XYZ + gyro XYZ + roll/pitch/yaw.
  if(incoming.length===20 && incoming[0]===0x55 && incoming[1]===0x61){
    const dv=new DataView(incoming.buffer,incoming.byteOffset,incoming.byteLength),g=9.80665;
    live.ax=dv.getInt16(2,true)/32768*16*g;
    live.ay=dv.getInt16(4,true)/32768*16*g;
    live.az=dv.getInt16(6,true)/32768*16*g;
    live.roll=dv.getInt16(14,true)/32768*180;
    live.pitch=dv.getInt16(16,true)/32768*180;
    live.yaw=dv.getInt16(18,true)/32768*180;
    if(liveEl)liveEl.textContent='LIVE • R '+live.roll.toFixed(1)+'°  P '+live.pitch.toFixed(1)+'°  A '+Math.hypot(live.ax,live.ay,live.az).toFixed(2)+' m/s²';
    integrate(live.ax,live.ay,live.az);
    return;
  }
  // Legacy 11-byte WIT frames, retained for compatibility.
  rx.push(...incoming);
  while(rx.length>=11){
    const s=rx.indexOf(0x55);if(s<0){rx=[];break}if(s>0)rx.splice(0,s);if(rx.length<11)break;
    const type=rx[1];if(type<0x50||type>0x5f){rx.shift();continue}
    const frame=rx.slice(0,11);rx.splice(0,11);const dv=new DataView(Uint8Array.from(frame).buffer);
    if(type===0x51){const g=9.80665;live.ax=dv.getInt16(2,true)/32768*16*g;live.ay=dv.getInt16(4,true)/32768*16*g;live.az=dv.getInt16(6,true)/32768*16*g;integrate(live.ax,live.ay,live.az)}
    else if(type===0x53){live.roll=dv.getInt16(2,true)/32768*180;live.pitch=dv.getInt16(4,true)/32768*180;live.yaw=dv.getInt16(6,true)/32768*180}
  }
  if(liveEl)liveEl.textContent='RX '+incoming.length+' bytes • '+Array.from(incoming.slice(0,8)).map(x=>x.toString(16).padStart(2,'0')).join(' ');
}
function integrate(ax,ay,az){if(!running)return;let now=performance.now();if(!last){last=now;return}let dt=Math.min((now-last)/1000,.05);last=now;let mag=Math.hypot(ax,ay,az),still=Math.abs(mag-9.80665)<.12;if(still){if(!lastStill)lastStill=now;if(now-lastStill>180){vel.x=vel.y=vel.z=0}return}else lastStill=0;
// Raw prototype: gravity/orientation compensation will be calibrated from full IMU packets.
vel.x+=ax*dt;vel.y+=ay*dt;vel.z+=(az-9.80665)*dt;pos.x+=vel.x*dt;pos.y+=vel.y*dt;pos.z+=vel.z*dt;render()}
$('start').onclick=()=>{pos={x:0,y:0,z:0};vel={x:0,y:0,z:0};pts=[{...pos}];last=0;running=true;$('point').disabled=$('close').disabled=false;render()};
$('point').onclick=()=>{pts.push({...pos});render()};$('close').onclick=()=>{if(pts.length>2)pts.push({...pts[0]});render()};$('reset').onclick=()=>{running=false;pts=[];pos={x:0,y:0,z:0};vel={x:0,y:0,z:0};render()};
function earthwork(areaFt2){const t=parseFloat(document.getElementById('targetGrade')?.value||0);if(!pts.length||areaFt2<=0)return {cut:0,fill:0,net:0};let diffs=pts.map(p=>p.z*3.28084-t),cutDepth=diffs.filter(x=>x>0),fillDepth=diffs.filter(x=>x<0).map(Math.abs);let cutAvg=cutDepth.length?cutDepth.reduce((a,b)=>a+b,0)/diffs.length:0,fillAvg=fillDepth.length?fillDepth.reduce((a,b)=>a+b,0)/diffs.length:0;let cut=areaFt2*cutAvg/27,fill=areaFt2*fillAvg/27;return {cut,fill,net:fill-cut}}
function render(){let path=[...pts,...(running?[pos]:[])],dist=0;for(let i=1;i<path.length;i++)dist+=Math.hypot(path[i].x-path[i-1].x,path[i].y-path[i-1].y,path[i].z-path[i-1].z);let horiz=Math.hypot(pos.x,pos.y),ft=3.28084;$('distance').textContent=(dist*ft).toFixed(2)+' ft';$('elevation').textContent=(pos.z*ft).toFixed(2)+' ft';$('slope').textContent=(horiz?pos.z/horiz*100:0).toFixed(1)+'%';let area=0;if(pts.length>2)for(let i=0,j=pts.length-1;i<pts.length;j=i++)area+=(pts[j].x*pts[i].y-pts[i].x*pts[j].y);area=Math.abs(area/2)*10.7639;$('area').textContent=area.toFixed(0)+' ft²';let ew=earthwork(area);let cutEl=document.getElementById('cut'),fillEl=document.getElementById('fill'),netEl=document.getElementById('net');if(cutEl){cutEl.textContent=ew.cut.toFixed(1)+' yd³';fillEl.textContent=ew.fill.toFixed(1)+' yd³';netEl.textContent=Math.abs(ew.net)<.05?'Balanced':(ew.net>0?'IMPORT '+ew.net.toFixed(1)+' yd³':'EXPORT '+Math.abs(ew.net).toFixed(1)+' yd³')}$('points').innerHTML=pts.map((p,i)=>'<div class="point"><span>P'+(i+1)+'</span><span>'+((p.x)*ft).toFixed(1)+', '+((p.y)*ft).toFixed(1)+', '+((p.z)*ft).toFixed(1)+' ft</span></div>').join('');draw(path)}
function draw(path){let c=$('map'),r=c.getBoundingClientRect(),d=devicePixelRatio||1;c.width=r.width*d;c.height=r.height*d;let x=c.getContext('2d');x.scale(d,d);x.fillStyle='#fff';x.fillRect(0,0,r.width,r.height);x.strokeStyle='#e5e7eb';for(let i=0;i<r.width;i+=40){x.beginPath();x.moveTo(i,0);x.lineTo(i,r.height);x.stroke()}for(let i=0;i<r.height;i+=40){x.beginPath();x.moveTo(0,i);x.lineTo(r.width,i);x.stroke()}if(!path.length)return;let xs=path.map(p=>p.x),ys=path.map(p=>p.y),minx=Math.min(...xs),maxx=Math.max(...xs),miny=Math.min(...ys),maxy=Math.max(...ys),scale=Math.min((r.width-50)/(maxx-minx||1),(r.height-50)/(maxy-miny||1));x.strokeStyle='#111827';x.lineWidth=3;x.beginPath();path.forEach((p,i)=>{let px=25+(p.x-minx)*scale,py=r.height-25-(p.y-miny)*scale;i?x.lineTo(px,py):x.moveTo(px,py)});x.stroke()}render();
document.getElementById('targetGrade')?.addEventListener('input',render);
