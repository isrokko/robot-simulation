// Built-in sample robots (URDF). Primitive geometry only so they need no mesh files.
const cyl = (r, l, z) => `<collision><origin xyz="0 0 ${z}"/><geometry><cylinder radius="${r}" length="${l}"/></geometry></collision>`;
const inert = (m, z, i) => `<inertial><origin xyz="0 0 ${z}"/><mass value="${m}"/><inertia ixx="${i}" iyy="${i}" izz="${i / 4}" ixy="0" ixz="0" iyz="0"/></inertial>`;

function quadLeg(n, x, y, effort) {
  return `
  <link name="${n}_hip">${inert(0.4, 0, 0.0004)}<collision><geometry><sphere radius="0.04"/></geometry></collision></link>
  <joint name="${n}_hip_roll" type="revolute"><parent link="body"/><child link="${n}_hip"/>
    <origin xyz="${x} ${y} 0"/><axis xyz="1 0 0"/><limit lower="-0.6" upper="0.6" effort="${effort}" velocity="20"/></joint>
  <link name="${n}_thigh">${inert(0.8, -0.1, 0.003)}${cyl(0.025, 0.2, -0.1)}</link>
  <joint name="${n}_hip_pitch" type="revolute"><parent link="${n}_hip"/><child link="${n}_thigh"/>
    <origin xyz="0 0 0"/><axis xyz="0 1 0"/><limit lower="-1.5" upper="1.5" effort="${effort}" velocity="20"/></joint>
  <link name="${n}_shin">${inert(0.25, -0.1, 0.001)}${cyl(0.018, 0.2, -0.1)}</link>
  <joint name="${n}_knee" type="revolute"><parent link="${n}_thigh"/><child link="${n}_shin"/>
    <origin xyz="0 0 -0.2"/><axis xyz="0 1 0"/><limit lower="-2.4" upper="0.1" effort="${effort}" velocity="20"/></joint>
  <link name="${n}_foot">${inert(0.05, 0, 0.00001)}<collision><geometry><sphere radius="0.025"/></geometry></collision></link>
  <joint name="${n}_foot_fix" type="fixed"><parent link="${n}_shin"/><child link="${n}_foot"/><origin xyz="0 0 -0.2"/></joint>`;
}

export const QUAD = `<?xml version="1.0"?>
<robot name="sample_quadruped">
  <link name="body">${inert(6, 0, 0.05)}<collision><geometry><box size="0.5 0.24 0.1"/></geometry></collision></link>
  ${quadLeg('FL', 0.22, 0.13, 18)}${quadLeg('FR', 0.22, -0.13, 18)}
  ${quadLeg('RL', -0.22, 0.13, 18)}${quadLeg('RR', -0.22, -0.13, 18)}
</robot>`;

function bipedLeg(s, y) {
  const j = (name, p, c, axis, lo, hi, eff, xyz = '0 0 0') =>
    `<joint name="${s}_${name}" type="revolute"><parent link="${p}"/><child link="${c}"/><origin xyz="${xyz}"/><axis xyz="${axis}"/><limit lower="${lo}" upper="${hi}" effort="${eff}" velocity="15"/></joint>`;
  return `
  <link name="${s}_hip1">${inert(0.3, 0, 0.0003)}</link>
  <link name="${s}_hip2">${inert(0.3, 0, 0.0003)}</link>
  <link name="${s}_thigh">${inert(2.0, -0.18, 0.02)}${cyl(0.045, 0.36, -0.18)}</link>
  <link name="${s}_shank">${inert(1.2, -0.18, 0.012)}${cyl(0.035, 0.36, -0.18)}</link>
  <link name="${s}_ankle">${inert(0.15, 0, 0.0001)}</link>
  <link name="${s}_foot">${inert(0.5, -0.03, 0.001)}<collision><origin xyz="0.03 0 -0.03"/><geometry><box size="0.22 0.1 0.03"/></geometry></collision></link>
  ${j('hip_yaw', 'pelvis', s + '_hip1', '0 0 1', -0.5, 0.5, 40, `0 ${y} -0.06`)}
  ${j('hip_roll', s + '_hip1', s + '_hip2', '1 0 0', -0.4, 0.4, 60)}
  ${j('hip_pitch', s + '_hip2', s + '_thigh', '0 1 0', -1.6, 1.0, 60)}
  ${j('knee', s + '_thigh', s + '_shank', '0 1 0', 0, 2.2, 60, '0 0 -0.36')}
  ${j('ankle_pitch', s + '_shank', s + '_ankle', '0 1 0', -0.8, 0.8, 25, '0 0 -0.36')}
  ${j('ankle_roll', s + '_ankle', s + '_foot', '1 0 0', -0.4, 0.4, 15)}`;
}
function arm(s, y) {
  return `
  <link name="${s}_upper_arm">${inert(0.8, -0.13, 0.004)}${cyl(0.03, 0.26, -0.13)}</link>
  <link name="${s}_forearm">${inert(0.5, -0.12, 0.002)}${cyl(0.025, 0.24, -0.12)}</link>
  <joint name="${s}_shoulder_pitch" type="revolute"><parent link="torso"/><child link="${s}_upper_arm"/><origin xyz="0 ${y} 0.18"/><axis xyz="0 1 0"/><limit lower="-2" upper="2" effort="15" velocity="10"/></joint>
  <joint name="${s}_elbow" type="revolute"><parent link="${s}_upper_arm"/><child link="${s}_forearm"/><origin xyz="0 0 -0.26"/><axis xyz="0 1 0"/><limit lower="-2" upper="0" effort="8" velocity="10"/></joint>`;
}

export const BIPED = `<?xml version="1.0"?>
<robot name="sample_humanoid">
  <link name="pelvis">${inert(4, 0, 0.03)}<collision><geometry><box size="0.16 0.26 0.12"/></geometry></collision></link>
  <link name="torso">${inert(9, 0.12, 0.12)}<collision><origin xyz="0 0 0.14"/><geometry><box size="0.18 0.3 0.3"/></geometry></collision></link>
  <joint name="waist" type="fixed"><parent link="pelvis"/><child link="torso"/><origin xyz="0 0 0.1"/></joint>
  <link name="head">${inert(1.5, 0, 0.005)}<collision><geometry><sphere radius="0.09"/></geometry></collision></link>
  <joint name="neck" type="fixed"><parent link="torso"/><child link="head"/><origin xyz="0 0 0.42"/></joint>
  ${bipedLeg('L', 0.09)}${bipedLeg('R', -0.09)}
  ${arm('L', 0.2)}${arm('R', -0.2)}
</robot>`;
