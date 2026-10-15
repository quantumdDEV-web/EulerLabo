import { describe, expect, it } from 'vitest';
import { determinant3, eulerToMatrix, matrixToEuler, mul3v, norm, normalizeQ, eulerToQuaternion, quaternionToMatrix, transformPoint, inversePoint, makeTransform, identity3, cross, eulerRatesToBodyOmega } from '../src/physics/math';
const close=(a:number,b:number,eps=1e-9)=>expect(Math.abs(a-b)).toBeLessThan(eps);
describe('rigid-body kinematics math',()=>{
 it('returns identity for zero Euler angles',()=>eulerToMatrix(0,0,0).forEach((v,i)=>close(v,identity3[i]!)));
 it('rotates by signed quarter turns about each principal axis',()=>{
   const cases:[number,number,number,readonly [number,number,number],readonly [number,number,number]][]=[
     [Math.PI/2,0,0,[0,1,0],[0,0,1]],[-Math.PI/2,0,0,[0,1,0],[0,0,-1]],
     [0,Math.PI/2,0,[0,0,1],[1,0,0]],[0,-Math.PI/2,0,[0,0,1],[-1,0,0]],
     [0,0,Math.PI/2,[1,0,0],[0,1,0]],[0,0,-Math.PI/2,[1,0,0],[0,-1,0]]
   ];
   for(const [roll,pitch,yaw,v,w] of cases)mul3v(eulerToMatrix(roll,pitch,yaw),v).forEach((n,j)=>close(n,w[j]!,1e-8));
 });
 it('creates a proper orthogonal rotation and round trips Euler angles away from singularity',()=>{const r=eulerToMatrix(.31,-.72,1.13);close(determinant3(r),1);for(let c=0;c<3;c++){const col=[r[c]!,r[3+c]!,r[6+c]!] as const;close(norm(col),1);for(let d=c+1;d<3;d++)close(col[0]*r[d]+col[1]*r[3+d]+col[2]*r[6+d],0);}const e=matrixToEuler(r);close(e.roll,.31);close(e.pitch,-.72);close(e.yaw,1.13);});
 it('handles gimbal lock with a documented yaw-zero representative',()=>{const r=eulerToMatrix(.4,Math.PI/2,-.2),e=matrixToEuler(r);expect(e.singular).toBe(true);close(e.yaw,0);const reconstructed=eulerToMatrix(e.roll,e.pitch,e.yaw);r.forEach((v,i)=>close(v,reconstructed[i]!,1e-8));});
 it('normalizes quaternions and matches matrix orientation',()=>{const q=normalizeQ([2,1,-3,.5]);close(Math.hypot(...q),1);const e=[.4,.2,-1.1] as const,a=eulerToMatrix(...e),b=quaternionToMatrix(eulerToQuaternion(...e));a.forEach((v,i)=>close(v,b[i]!));});
 it('round trips a point from body coordinates to world and back',()=>{const r=eulerToMatrix(.3,.8,-.6),t=[2,-1,4] as const,p=[-.2,1.4,2.2] as const;const w=transformPoint(r,t,p),back=inversePoint(r,t,w);back.forEach((v,i)=>close(v,p[i]!));expect(makeTransform(r,t)).toHaveLength(16);});
 it('preserves vector lengths and computes cross products',()=>{const v=[2,-3,4] as const,r=eulerToMatrix(.9,-.3,1.7);close(norm(mul3v(r,v)),norm(v));expect(cross([1,0,0],[0,1,0])).toEqual([0,0,1]);});
 it('maps Euler rates to body angular velocity using the documented formula',()=>expect(eulerRatesToBodyOmega(0,0,2,3,4)).toEqual([2,3,4]));
});
