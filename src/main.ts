import './style.css';
import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { eulerToMatrix, matrixToEuler, eulerToQuaternion, composePose, transformPoint, inversePoint, mul3, type Mat3, type Pose } from './physics/math';

const app=document.querySelector<HTMLDivElement>('#app')!;
app.innerHTML=`<header><div><div class="eyebrow">APPLIED MECHANICS · INTERACTIVE LAB</div><h1>Rigid body <span>laboratory</span></h1></div><div class="header-note"><b>KINEMATICS</b><small>Pose and frame transformations</small></div></header><main><section class="viewport"><div id="scene"></div><div class="scene-tag">WORLD FRAME <span>·</span> BODY FRAME</div><div class="scene-help">Drag to orbit · Scroll to zoom · Right-drag to pan</div><div class="scene-actions"><button id="camera-reset" class="icon-button">Reset camera</button></div></section><aside><section class="card pose-card"><div class="section-title"><div><div class="eyebrow">01 / GENERALIZED COORDINATES</div><h2>Prescribed pose</h2></div><button id="pose-reset" class="text-button">Reset pose</button></div><div id="controls"></div><label class="check"><input id="degrees" type="checkbox" checked> Display angles in degrees</label><div class="shape-row"><label for="shape">Geometry</label><select id="shape"><option value="box">Rectangular body</option><option value="sphere">Sphere</option><option value="cylinder">Cylinder</option><option value="asymmetric">Asymmetric body</option></select></div></section><section class="card toggles"><div class="eyebrow">02 / SCENE LAYERS</div><div class="toggle-grid"><label><input type="checkbox" id="world-toggle" checked> World axes</label><label><input type="checkbox" id="body-toggle" checked> Body axes</label><label><input type="checkbox" id="vector-toggle" checked> Position vector</label><label><input type="checkbox" id="grid-toggle" checked> Ground grid</label></div></section><section class="card math-card"><div class="section-title"><div><div class="eyebrow">03 / TRANSFORM</div><h2>Coordinate map</h2></div><span class="badge">BODY → WORLD</span></div><div class="equation">p<sub>W</sub> = R<sub>WB</sub> p<sub>B</sub> + t<sub>WB</sub></div><div class="matrix-heading">ROTATION MATRIX R<sub>WB</sub></div><pre id="matrix"></pre><div class="matrix-heading">UNIT QUATERNION [w, x, y, z]</div><pre id="quaternion"></pre><div class="matrix-heading">HOMOGENEOUS TRANSFORM T<sub>WB</sub></div><pre id="transform"></pre></section></aside></main><section class="below"><section class="card euler-card"><div class="section-title"><div><div class="eyebrow">04 / ROTATION COMPOSITION</div><h2>Euler angles are order dependent</h2></div><span class="badge">Z → Y → X · ACTIVE</span></div><p>Yaw about fixed Z, then pitch about fixed Y, then roll about fixed X gives <code>R<sub>WB</sub> = R<sub>z</sub>R<sub>y</sub>R<sub>x</sub></code>. Intermediate body frames below show the cumulative orientation after each multiplication.</p><div id="steps" class="steps"></div><div class="warning"><b>Gimbal lock</b><span id="singularity">When pitch approaches ±90°, the Euler parameterization becomes singular. The rigid body itself retains all three rotational degrees of freedom.</span></div></section><section class="card vector-card"><div class="eyebrow">05 / FRAME CONVERSION</div><h2>Transform a body point</h2><p>Enter a local point to see the translated world point and its inverse transformation.</p><div class="point-fields" id="point-inputs"></div><div class="equation result-equation">p<sub>W</sub> = <span id="world-point"></span></div><div class="equation result-equation">p<sub>B</sub> recovered = <span id="body-point"></span></div><div class="fine-print">Points include translation. Free vectors rotate without translation. Angular velocity is a physical vector and is distinct from Euler angle rates.</div></section></section><footer>SI units · Internally radians · Prescribed-pose kinematics only</footer>`;

const sceneEl=document.querySelector<HTMLDivElement>('#scene')!;
const scene=new THREE.Scene(); scene.background=new THREE.Color('#101a27');
const camera=new THREE.PerspectiveCamera(43,1,.1,100); camera.position.set(7,6.3,9);
const renderer=new THREE.WebGLRenderer({antialias:true}); renderer.setPixelRatio(Math.min(devicePixelRatio,2)); renderer.outputColorSpace=THREE.SRGBColorSpace; sceneEl.append(renderer.domElement);
const orbit=new OrbitControls(camera,renderer.domElement); orbit.target.set(0,1,0); orbit.enableDamping=true;
scene.add(new THREE.HemisphereLight(0xd5e7ff,0x263447,2.1)); const key=new THREE.DirectionalLight(0xffffff,3); key.position.set(4,8,5); scene.add(key);
const grid=new THREE.GridHelper(20,20,0x42556a,0x28394d); scene.add(grid);
const worldAxes=new THREE.AxesHelper(2.2); scene.add(worldAxes);
const origin=new THREE.Mesh(new THREE.SphereGeometry(.07,16,12),new THREE.MeshBasicMaterial({color:0xf3f6fa})); scene.add(origin);
const body=new THREE.Group(); scene.add(body); const material=new THREE.MeshStandardMaterial({color:0x4eb9a5,metalness:.18,roughness:.36}); let solid:THREE.Mesh; let ghostGroup=new THREE.Group(); scene.add(ghostGroup);
function geometry(type:string):THREE.BufferGeometry {
 if(type==='sphere')return new THREE.SphereGeometry(.85,32,24);
 if(type==='cylinder')return new THREE.CylinderGeometry(.62,.62,1.55,32);
 if(type==='asymmetric'){const g=new THREE.BufferGeometry(); const v=new Float32Array([-.45,-.5,-.35,.7,-.4,-.3,.25,.65,-.25,-.3,-.3,.6,.55,-.15,.5,-.2,.6,.4]); g.setAttribute('position',new THREE.BufferAttribute(v,3)); g.setIndex([0,1,2,0,3,1,1,3,4,1,4,2,2,4,5,2,5,0,0,5,3,3,5,4]); g.computeVertexNormals(); return g; }
 return new THREE.BoxGeometry(1.55,1.25,1.05);
}
function setShape(type:string){if(solid)body.remove(solid); solid=new THREE.Mesh(geometry(type),material); solid.geometry.computeVertexNormals(); solid.castShadow=true; solid.receiveShadow=true; body.add(solid); const edges=new THREE.LineSegments(new THREE.EdgesGeometry(solid.geometry),new THREE.LineBasicMaterial({color:0xc9fff4,transparent:true,opacity:.38})); solid.add(edges);}
setShape('box');
const bodyAxes=new THREE.AxesHelper(1.65); body.add(bodyAxes);
const positionLine=new THREE.Line(new THREE.BufferGeometry(),new THREE.LineBasicMaterial({color:0xf5bf63})); scene.add(positionLine);
const controlsEl=document.querySelector<HTMLDivElement>('#controls')!;
const keys=['x','y','z','roll','pitch','yaw'] as const;
const symbols=['x','y','z','φ roll','θ pitch','ψ yaw'];
const state:Pose={position:[0,1,0],roll:0,pitch:0,yaw:0};
const fields=new Map<string,{range:HTMLInputElement;number:HTMLInputElement}>();
controlsEl.innerHTML=keys.map((k,i)=>`<div class="control"><div class="control-head"><label for="${k}-num"><span class="coord-dot c${i%3}"></span>${symbols[i]}</label><div class="control-entry"><input id="${k}-num" class="num" type="number" step="${i<3?'.1':'.01'}"/><span class="unit" id="${k}-unit">${i<3?'m':'°'}</span></div></div><input id="${k}-range" class="range" type="range" min="${i<3?'-5':'-180'}" max="${i<3?'5':'180'}" step="${i<3?'.01':'.1'}"/></div>`).join('');
keys.forEach(k=>fields.set(k,{range:document.querySelector<HTMLInputElement>(`#${k}-range`)!,number:document.querySelector<HTMLInputElement>(`#${k}-num`)!}));
const degreeInput=document.querySelector<HTMLInputElement>('#degrees')!;
function angleIn(key:string):number {return key==='roll'?state.roll:key==='pitch'?state.pitch:state.yaw;}
function displayValue(key:string){return keys.indexOf(key as typeof keys[number])<3?(state.position[keys.indexOf(key as typeof keys[number])]!).toFixed(2):(angleIn(key)*180/Math.PI).toFixed(1);}
function setValue(key:string,value:number){if(key==='x'||key==='y'||key==='z'){const p=[...state.position] as [number,number,number];p[key==='x'?0:key==='y'?1:2]=value;state.position=p;}else {const angle=degreeInput.checked?value*Math.PI/180:value;if(key==='roll')state.roll=angle;else if(key==='pitch')state.pitch=angle;else state.yaw=angle;}refresh();updatePoint();}
fields.forEach((els,k)=>{els.range.addEventListener('input',()=>setValue(k,Number(els.range.value)));els.number.addEventListener('change',()=>setValue(k,Number(els.number.value)));});
function matrixRows(m:readonly number[],n:number,digits=3){return Array.from({length:n},(_,r)=>`[ ${Array.from({length:n},(_,c)=>m[r*n+c]!.toFixed(digits).padStart(7)).join('  ')} ]`).join('\n');}
const matrixEl=document.querySelector<HTMLPreElement>('#matrix')!,quatEl=document.querySelector<HTMLPreElement>('#quaternion')!,transformEl=document.querySelector<HTMLPreElement>('#transform')!;
const stepsEl=document.querySelector<HTMLDivElement>('#steps')!;
function basisGroup(r:Mat3,opacity:number){const g=new THREE.Group(); const axes=new THREE.AxesHelper(.9);for(const m of Array.isArray(axes.material)?axes.material:[axes.material]){m.transparent=true;m.opacity=opacity;}g.add(axes);g.quaternion.setFromRotationMatrix(new THREE.Matrix4().set(r[0],r[1],r[2],0,r[3],r[4],r[5],0,r[6],r[7],r[8],0,0,0,0,1)); return g;}
function refresh(){const r=eulerToMatrix(state.roll,state.pitch,state.yaw),t=state.position;body.position.set(...t);body.rotation.set(state.roll,state.pitch,state.yaw,'ZYX');
 positionLine.geometry.dispose();positionLine.geometry=new THREE.BufferGeometry().setFromPoints([new THREE.Vector3(0,0,0),new THREE.Vector3(...t)]);
 worldAxes.visible=document.querySelector<HTMLInputElement>('#world-toggle')!.checked;bodyAxes.visible=document.querySelector<HTMLInputElement>('#body-toggle')!.checked;positionLine.visible=document.querySelector<HTMLInputElement>('#vector-toggle')!.checked;grid.visible=document.querySelector<HTMLInputElement>('#grid-toggle')!.checked;
 keys.forEach((k,i)=>{const value=displayValue(k),el=fields.get(k)!;el.number.value=value;el.range.value=String(i<3?Number(value):Number(value));document.querySelector(`#${k}-unit`)!.textContent=i<3?'m':degreeInput.checked?'°':'rad';if(i>=3&&!degreeInput.checked){el.range.min='-3.14159';el.range.max='3.14159';el.range.step='.001';}else if(i>=3){el.range.min='-180';el.range.max='180';el.range.step='.1';}});
 matrixEl.textContent=matrixRows(r,3); const q=eulerToQuaternion(state.roll,state.pitch,state.yaw);quatEl.textContent=`[ ${q.map(v=>v.toFixed(5)).join(', ')} ]`;transformEl.textContent=matrixRows(composePose(state),4);
 const rz:Mat3=[Math.cos(state.yaw),-Math.sin(state.yaw),0,Math.sin(state.yaw),Math.cos(state.yaw),0,0,0,1],ry:Mat3=[Math.cos(state.pitch),0,Math.sin(state.pitch),0,1,0,-Math.sin(state.pitch),0,Math.cos(state.pitch)],rx:Mat3=[1,0,0,0,Math.cos(state.roll),-Math.sin(state.roll),0,Math.sin(state.roll),Math.cos(state.roll)];
 const rs=[rz,mul3(rz,ry),mul3(mul3(rz,ry),rx)],names=['After yaw Rz(ψ)','After pitch Ry(θ) Rz(ψ)Ry(θ)','After roll Rx(φ) Rz(ψ)Ry(θ)Rx(φ)'];
 ghostGroup.clear();stepsEl.innerHTML='';rs.forEach((m,i)=>{const group=basisGroup(m,.62-i*.12);group.position.set(-2.65+i*2.65,2.6,-2.15);ghostGroup.add(group);const card=document.createElement('div');card.className='step';card.innerHTML=`<b>${names[i]}</b><pre>${matrixRows(m,3,2)}</pre>`;stepsEl.append(card);});
 const e=matrixToEuler(r);document.querySelector('#singularity')!.textContent=e.singular?`Singular configuration detected (pitch ${(state.pitch*180/Math.PI).toFixed(1)}°). Roll and yaw are not separately identifiable from orientation; the rigid body retains all three physical rotational degrees of freedom.`:'As pitch approaches ±90°, the Euler parameterization becomes singular. The rigid body itself retains all three rotational degrees of freedom.';
}
refresh();
// A small local point makes point-vs-vector transformation observable without cluttering the view.
const pInputs=document.querySelector<HTMLDivElement>('#point-inputs')!;pInputs.innerHTML=['x','y','z'].map((axis,i)=>`<label>${axis}<input type="number" step=".1" value="${i===0?1:i===1?.4:.25}" id="point-${axis}"></label>`).join('');
function updatePoint(){const p=['x','y','z'].map(a=>Number(document.querySelector<HTMLInputElement>(`#point-${a}`)!.value)) as [number,number,number];const r=eulerToMatrix(state.roll,state.pitch,state.yaw),pw=transformPoint(r,state.position,p),pb=inversePoint(r,state.position,pw);document.querySelector('#world-point')!.textContent=`(${pw.map(v=>v.toFixed(3)).join(', ')}) m`;document.querySelector('#body-point')!.textContent=`(${pb.map(v=>v.toFixed(3)).join(', ')}) m`;}
pInputs.querySelectorAll('input').forEach(el=>el.addEventListener('input',updatePoint));
const originalRefresh=refresh; void originalRefresh; updatePoint();
// Refresh frame conversion when a pose control changes.
fields.forEach(({number,range})=>{number.addEventListener('input',updatePoint);number.addEventListener('change',updatePoint);range.addEventListener('input',updatePoint);});
degreeInput.addEventListener('change',()=>{keys.slice(3).forEach(k=>{const f=fields.get(k)!;f.range.value=displayValue(k);f.number.value=displayValue(k);});refresh();});
document.querySelectorAll<HTMLInputElement>('.toggles input').forEach(el=>el.addEventListener('change',refresh));
document.querySelector<HTMLInputElement>('#shape')!.addEventListener('change',e=>setShape((e.target as HTMLSelectElement).value));
document.querySelector('#pose-reset')!.addEventListener('click',()=>{state.position=[0,1,0];state.roll=state.pitch=state.yaw=0;refresh();updatePoint();});
document.querySelector('#camera-reset')!.addEventListener('click',()=>{camera.position.set(7,6.3,9);orbit.target.set(0,1,0);orbit.update();});
function resize(){const w=sceneEl.clientWidth,h=sceneEl.clientHeight;renderer.setSize(w,h,false);camera.aspect=w/h;camera.updateProjectionMatrix();}new ResizeObserver(resize).observe(sceneEl);resize();
function frame(){requestAnimationFrame(frame);orbit.update();renderer.render(scene,camera);}frame();
