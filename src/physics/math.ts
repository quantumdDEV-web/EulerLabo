export type Vec3 = readonly [number, number, number];
export type Mat3 = readonly [number, number, number, number, number, number, number, number, number];
export type Mat4 = readonly number[];
export type Quaternion = readonly [number, number, number, number]; // [w,x,y,z]
export interface Pose { position: Vec3; roll: number; pitch: number; yaw: number }

export const add = (a: Vec3, b: Vec3): Vec3 => [a[0]+b[0],a[1]+b[1],a[2]+b[2]];
export const sub = (a: Vec3, b: Vec3): Vec3 => [a[0]-b[0],a[1]-b[1],a[2]-b[2]];
export const scale = (a: Vec3, k: number): Vec3 => [a[0]*k,a[1]*k,a[2]*k];
export const dot = (a: Vec3,b: Vec3): number => a[0]*b[0]+a[1]*b[1]+a[2]*b[2];
export const cross = (a: Vec3,b: Vec3): Vec3 => [a[1]*b[2]-a[2]*b[1],a[2]*b[0]-a[0]*b[2],a[0]*b[1]-a[1]*b[0]];
export const norm = (a: Vec3): number => Math.sqrt(dot(a,a));
export function normalize(a: Vec3): Vec3 { const n=norm(a); if (n===0) throw new RangeError('Cannot normalize a zero vector'); return scale(a,1/n); }
export const identity3: Mat3 = [1,0,0,0,1,0,0,0,1];
export function mul3v(m: Mat3,v: Vec3): Vec3 { return [m[0]*v[0]+m[1]*v[1]+m[2]*v[2],m[3]*v[0]+m[4]*v[1]+m[5]*v[2],m[6]*v[0]+m[7]*v[1]+m[8]*v[2]]; }
export function mul3(a: Mat3,b: Mat3): Mat3 { const o:number[]=[]; for(let r=0;r<3;r++) for(let c=0;c<3;c++) o.push(a[r*3]!*b[c]!+a[r*3+1]!*b[3+c]!+a[r*3+2]!*b[6+c]!); return o as unknown as Mat3; }
export function transpose3(m: Mat3): Mat3 { return [m[0],m[3],m[6],m[1],m[4],m[7],m[2],m[5],m[8]]; }
export function determinant3(m: Mat3): number { return m[0]*(m[4]*m[8]-m[5]*m[7])-m[1]*(m[3]*m[8]-m[5]*m[6])+m[2]*(m[3]*m[7]-m[4]*m[6]); }
export function eulerToMatrix(roll:number,pitch:number,yaw:number):Mat3 {
 const c1=Math.cos(roll),s1=Math.sin(roll),c2=Math.cos(pitch),s2=Math.sin(pitch),c3=Math.cos(yaw),s3=Math.sin(yaw);
 return [c3*c2,c3*s2*s1-s3*c1,c3*s2*c1+s3*s1,s3*c2,s3*s2*s1+c3*c1,s3*s2*c1-c3*s1,-s2,c2*s1,c2*c1];
}
export function matrixToEuler(m:Mat3):{roll:number;pitch:number;yaw:number;singular:boolean} {
 const pitch=Math.asin(Math.max(-1,Math.min(1,-m[6]))), singular=Math.abs(Math.cos(pitch))<1e-8;
 // At singularity choose yaw=0; roll absorbs the observable combined rotation.
 return singular ? {roll:Math.atan2(-m[5],m[4]),pitch,yaw:0,singular} : {roll:Math.atan2(m[7],m[8]),pitch,yaw:Math.atan2(m[3],m[0]),singular};
}
export function normalizeQ(q:Quaternion):Quaternion { const n=Math.hypot(...q); if(n===0) throw new RangeError('Cannot normalize a zero quaternion'); return [q[0]/n,q[1]/n,q[2]/n,q[3]/n]; }
export function multiplyQ(a:Quaternion,b:Quaternion):Quaternion { const [w,x,y,z]=a,[v,i,j,k]=b; return [w*v-x*i-y*j-z*k,w*i+x*v+y*k-z*j,w*j-x*k+y*v+z*i,w*k+x*j-y*i+z*v]; }
export function eulerToQuaternion(roll:number,pitch:number,yaw:number):Quaternion { const h=roll/2,p=pitch/2,y=yaw/2; return normalizeQ([Math.cos(y)*Math.cos(p)*Math.cos(h)+Math.sin(y)*Math.sin(p)*Math.sin(h),Math.cos(y)*Math.cos(p)*Math.sin(h)-Math.sin(y)*Math.sin(p)*Math.cos(h),Math.cos(y)*Math.sin(p)*Math.cos(h)+Math.sin(y)*Math.cos(p)*Math.sin(h),Math.sin(y)*Math.cos(p)*Math.cos(h)-Math.cos(y)*Math.sin(p)*Math.sin(h)]); }
export function quaternionToMatrix(q0:Quaternion):Mat3 { const [w,x,y,z]=normalizeQ(q0); return [1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w),2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w),2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]; }
export function makeTransform(r:Mat3,t:Vec3):Mat4 { return [r[0],r[1],r[2],t[0],r[3],r[4],r[5],t[1],r[6],r[7],r[8],t[2],0,0,0,1]; }
export function transformPoint(r:Mat3,t:Vec3,p:Vec3):Vec3 { return add(mul3v(r,p),t); }
export function transformVector(r:Mat3,v:Vec3):Vec3 { return mul3v(r,v); }
export function inversePoint(r:Mat3,t:Vec3,p:Vec3):Vec3 { return mul3v(transpose3(r),sub(p,t)); }
export function composePose(p:Pose):Mat4 { return makeTransform(eulerToMatrix(p.roll,p.pitch,p.yaw),p.position); }
export function eulerRatesToBodyOmega(roll:number,pitch:number,rollDot:number,pitchDot:number,yawDot:number):Vec3 { return [rollDot-yawDot*Math.sin(pitch),pitchDot*Math.cos(roll)+yawDot*Math.sin(roll)*Math.cos(pitch),-pitchDot*Math.sin(roll)+yawDot*Math.cos(roll)*Math.cos(pitch)]; }
