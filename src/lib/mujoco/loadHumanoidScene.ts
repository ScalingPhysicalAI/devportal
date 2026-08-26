import * as THREE from "three";

// Geometry-building logic adapted from the official MuJoCo WASM demo
// (https://github.com/zalo/mujoco_wasm, MIT licensed): walks the compiled
// MjModel's geoms/lights and rebuilds them as a three.js scene graph, one
// THREE.Group per MuJoCo body. Trimmed to what our single static model
// needs -- no textures, tendons, flex, or GUI/keyframe hooks.

// eslint-disable-next-line @typescript-eslint/no-explicit-any
type Mujoco = any;
// eslint-disable-next-line @typescript-eslint/no-explicit-any
type MjModel = any;

const MODEL_URL = "/mujoco/scene/humanoid.xml";
const WORKING_DIR = "/working";

export async function loadHumanoidModel(mujoco: Mujoco): Promise<MjModel> {
  const xmlText = await (await fetch(MODEL_URL)).text();
  const meshNames = Array.from(new Set([...xmlText.matchAll(/file="meshes\/([^"]+)"/g)].map((m) => m[1])));

  if (!mujoco.FS.analyzePath(WORKING_DIR).exists) {
    mujoco.FS.mkdir(WORKING_DIR);
    mujoco.FS.mount(mujoco.MEMFS, { root: "." }, WORKING_DIR);
  }
  mujoco.FS.mkdir(`${WORKING_DIR}/meshes`);

  await Promise.all(
    meshNames.map(async (name) => {
      const buf = await (await fetch(`/mujoco/scene/meshes/${name}`)).arrayBuffer();
      mujoco.FS.writeFile(`${WORKING_DIR}/meshes/${name}`, new Uint8Array(buf));
    })
  );

  mujoco.FS.writeFile(`${WORKING_DIR}/humanoid.xml`, xmlText);
  return mujoco.MjModel.mj_loadXML(`${WORKING_DIR}/humanoid.xml`);
}

/** Access a swizzled (MuJoCo Z-up -> three.js Y-up) vector at `index` into `target`. */
export function getPosition(buffer: Float32Array | Float64Array, index: number, target: THREE.Vector3): THREE.Vector3 {
  return target.set(buffer[index * 3 + 0], buffer[index * 3 + 2], -buffer[index * 3 + 1]);
}

/** Access a swizzled (MuJoCo Z-up -> three.js Y-up) quaternion at `index` into `target`. */
export function getQuaternion(buffer: Float32Array | Float64Array, index: number, target: THREE.Quaternion): THREE.Quaternion {
  return target.set(-buffer[index * 4 + 1], -buffer[index * 4 + 3], buffer[index * 4 + 2], -buffer[index * 4 + 0]);
}

export interface BuiltScene {
  root: THREE.Group;
  bodies: Record<number, THREE.Group & { bodyID: number }>;
  lights: THREE.Light[];
}

// Canvas-drawn grid (black blocks, light-blue lines) for the ground plane --
// the MJCF's own checker <texture> isn't sampled anywhere in this loader
// (materials are read as flat rgba only), so the floor's look is entirely
// up to this.
const GRID_TILE_METERS = 0.2; // 20x20cm blocks, sized against the robot's real-world scale
let gridFloorTexture: THREE.CanvasTexture | null = null;
function getGridFloorTexture(planeWidth: number, planeHeight: number): THREE.CanvasTexture {
  if (!gridFloorTexture) {
    const size = 256;
    const canvas = document.createElement("canvas");
    canvas.width = size;
    canvas.height = size;
    const ctx = canvas.getContext("2d")!;
    ctx.fillStyle = "#050608";
    ctx.fillRect(0, 0, size, size);
    ctx.strokeStyle = "#4fb3e8";
    ctx.lineWidth = 2;
    ctx.strokeRect(2, 2, size - 4, size - 4);
    gridFloorTexture = new THREE.CanvasTexture(canvas);
    gridFloorTexture.wrapS = THREE.RepeatWrapping;
    gridFloorTexture.wrapT = THREE.RepeatWrapping;
    gridFloorTexture.colorSpace = THREE.SRGBColorSpace;
  }
  gridFloorTexture.repeat.set(planeWidth / GRID_TILE_METERS, planeHeight / GRID_TILE_METERS);
  return gridFloorTexture;
}

// Our converted URDF assigns no <material> at all -- every geom just
// carries the same flat placeholder rgba, so there's no real per-part color
// to read. Fall back to a black/silver two-tone matching the robot's actual
// reference photo: neck/face, actuator housings, hands, the lift column
// just below the waist, and the wheel covers/tyres are black plastic; the
// wider base plate below the lift column and the rest of the structural
// shell/limb segments are brushed silver -- silver on either side of the
// black wheel covers so the tyres actually read against it. Matched per
// *mesh* rather than per body -- e.g. the shoulder body's own housing mesh
// is silver, but it carries a separate actuator_dummy mesh as another geom
// on that same body, and that part reads black in the photo -- so
// classifying by body alone can't reproduce it, only by mesh name can.
const BLACK_MESH_NAME = /actuator|neck_joint|face_cover|palm|finger|thumb_knuckle|servo_spacer_hand|wrist_cover|^wheels_|caster|wheelbase_lift/i;
// Dark enough to survive a large flat face pointed almost straight at the
// key light (confirmed via the base plate's top face: at 0.2 the diffuse
// response at a near-1 N.L angle still read as light grey -- a curved part
// like the head only ever catches that angle across a tiny highlight spot,
// but a big flat face catches it across its whole area).
const FALLBACK_BLACK = [0.04, 0.04, 0.04, 1] as const;
const FALLBACK_SILVER = [0.7, 0.7, 0.7, 1] as const;

/** Decodes a null-terminated name out of MjModel's `names` byte blob, starting at `adr`. */
function decodeName(namesArray: Uint8Array, textDecoder: TextDecoder, adr: number): string {
  let end = adr;
  while (end < namesArray.length && namesArray[end] !== 0) end++;
  return textDecoder.decode(namesArray.subarray(adr, end));
}

/** Builds a three.js scene graph (one Group per MuJoCo body) from a loaded MjModel. */
export function buildSceneFromModel(mujoco: Mujoco, model: MjModel): BuiltScene {
  const textDecoder = new TextDecoder("utf-8");
  const namesArray = new Uint8Array(model.names as ArrayBufferLike);

  const root = new THREE.Group();
  root.name = "MuJoCo Root";

  // eslint-disable-next-line @typescript-eslint/no-explicit-any
  const bodies: Record<number, any> = {};
  const meshGeometries: Record<number, THREE.BufferGeometry> = {};
  const baseLinkSplitGeometries: Record<string, { top: THREE.BufferGeometry; rest: THREE.BufferGeometry }> = {};
  const lights: THREE.Light[] = [];

  for (let g = 0; g < model.ngeom; g++) {
    if (!(model.geom_group[g] < 3)) continue; // hide collision-only geoms (group 3+), same as `simulate`

    const b = model.geom_bodyid[g];
    const type = model.geom_type[g];
    const size = [model.geom_size[g * 3 + 0], model.geom_size[g * 3 + 1], model.geom_size[g * 3 + 2]];
    const partName =
      type === mujoco.mjtGeom.mjGEOM_MESH.value
        ? decodeName(namesArray, textDecoder, model.name_meshadr[model.geom_dataid[g]])
        : bodies[b]?.name ?? "";

    if (!(b in bodies)) {
      const group = new THREE.Group() as THREE.Group & { bodyID: number };
      group.name = decodeName(namesArray, textDecoder, model.name_bodyadr[b]);
      group.bodyID = b;
      bodies[b] = group;
    }

    let geometry: THREE.BufferGeometry = new THREE.SphereGeometry(size[0] * 0.5);
    let planeWidth = 0;
    let planeHeight = 0;
    if (type === mujoco.mjtGeom.mjGEOM_SPHERE.value) {
      geometry = new THREE.SphereGeometry(size[0]);
    } else if (type === mujoco.mjtGeom.mjGEOM_CAPSULE.value) {
      geometry = new THREE.CapsuleGeometry(size[0], size[1] * 2.0, 20, 20);
    } else if (type === mujoco.mjtGeom.mjGEOM_CYLINDER.value) {
      geometry = new THREE.CylinderGeometry(size[0], size[0], size[1] * 2.0);
    } else if (type === mujoco.mjtGeom.mjGEOM_BOX.value) {
      geometry = new THREE.BoxGeometry(size[0] * 2.0, size[2] * 2.0, size[1] * 2.0);
    } else if (type === mujoco.mjtGeom.mjGEOM_PLANE.value) {
      // THREE.PlaneGeometry defaults to lying in the XY plane (facing +Z);
      // rotate it into the XZ plane so it lies flat as a ground plane in
      // this Y-up scene.
      planeWidth = size[0] > 0 ? size[0] * 2 : 20;
      planeHeight = size[1] > 0 ? size[1] * 2 : 20;
      geometry = new THREE.PlaneGeometry(planeWidth, planeHeight);
      geometry.rotateX(-Math.PI / 2);
    } else if (type === mujoco.mjtGeom.mjGEOM_MESH.value) {
      const meshID = model.geom_dataid[g];
      if (!(meshID in meshGeometries)) {
        const bufGeom = new THREE.BufferGeometry();

        const vertexBuffer = model.mesh_vert.subarray(
          model.mesh_vertadr[meshID] * 3,
          (model.mesh_vertadr[meshID] + model.mesh_vertnum[meshID]) * 3
        );
        // Swizzle MuJoCo's Z-up vertices into three.js's Y-up space.
        for (let v = 0; v < vertexBuffer.length; v += 3) {
          const tmp = vertexBuffer[v + 1];
          vertexBuffer[v + 1] = vertexBuffer[v + 2];
          vertexBuffer[v + 2] = -tmp;
        }

        const faceBuffer = model.mesh_face.subarray(
          model.mesh_faceadr[meshID] * 3,
          (model.mesh_faceadr[meshID] + model.mesh_facenum[meshID]) * 3
        );

        bufGeom.setAttribute("position", new THREE.BufferAttribute(vertexBuffer, 3));
        bufGeom.setIndex(Array.from(faceBuffer));
        bufGeom.computeVertexNormals();
        meshGeometries[meshID] = bufGeom;
      }
      geometry = meshGeometries[meshID];
      bodies[b].has_custom_mesh = true;

      // The base plate: black from directly above, silver everywhere else
      // (sides, underside) -- same black as the hands. Split into two real
      // sub-meshes (by each triangle's final, post geom-quat, up-facing-ness)
      // rather than one mesh with per-vertex colors -- vertex colors on this
      // mesh silently didn't render (confirmed correct in the three.js scene
      // graph via runtime inspection: right geometry, right material flag,
      // right buffer values -- yet no visible effect), so two flat-colored
      // meshes it is, the same approach already proven everywhere else here.
      if (partName === "base_link") {
        const cacheKey = `${meshID}`;
        if (!(cacheKey in baseLinkSplitGeometries)) {
          const geomQuat = getQuaternion(model.geom_quat, g, new THREE.Quaternion());
          const topIndices: number[] = [];
          const restIndices: number[] = [];
          const pA = new THREE.Vector3();
          const pB = new THREE.Vector3();
          const pC = new THREE.Vector3();
          const edge1 = new THREE.Vector3();
          const edge2 = new THREE.Vector3();
          const faceNormal = new THREE.Vector3();
          const posAttr = geometry.attributes.position;
          const idx = geometry.index!;
          for (let i = 0; i < idx.count; i += 3) {
            const i0 = idx.getX(i);
            const i1 = idx.getX(i + 1);
            const i2 = idx.getX(i + 2);
            pA.fromBufferAttribute(posAttr, i0).applyQuaternion(geomQuat);
            pB.fromBufferAttribute(posAttr, i1).applyQuaternion(geomQuat);
            pC.fromBufferAttribute(posAttr, i2).applyQuaternion(geomQuat);
            edge1.subVectors(pB, pA);
            edge2.subVectors(pC, pA);
            faceNormal.crossVectors(edge1, edge2).normalize();
            (faceNormal.y > 0.5 ? topIndices : restIndices).push(i0, i1, i2);
          }

          const topGeom = new THREE.BufferGeometry();
          topGeom.setAttribute("position", posAttr);
          topGeom.setIndex(topIndices);
          topGeom.computeVertexNormals();

          const restGeom = new THREE.BufferGeometry();
          restGeom.setAttribute("position", posAttr);
          restGeom.setIndex(restIndices);
          restGeom.computeVertexNormals();

          baseLinkSplitGeometries[cacheKey] = { top: topGeom, rest: restGeom };
        }

        const { top, rest } = baseLinkSplitGeometries[cacheKey];
        for (const [geom, rgba] of [
          [top, FALLBACK_BLACK],
          [rest, FALLBACK_SILVER],
        ] as const) {
          const splitMesh = new THREE.Mesh(
            geom,
            new THREE.MeshStandardMaterial({ color: new THREE.Color(rgba[0], rgba[1], rgba[2]), roughness: 0.35, metalness: 0.25 })
          );
          splitMesh.castShadow = true;
          splitMesh.receiveShadow = true;
          (splitMesh as unknown as { bodyID: number }).bodyID = b;
          getPosition(model.geom_pos, g, splitMesh.position);
          getQuaternion(model.geom_quat, g, splitMesh.quaternion);
          bodies[b].add(splitMesh);
        }
        continue;
      }
    }

    let color: readonly [number, number, number, number] = [
      model.geom_rgba[g * 4 + 0],
      model.geom_rgba[g * 4 + 1],
      model.geom_rgba[g * 4 + 2],
      model.geom_rgba[g * 4 + 3],
    ];
    if (model.geom_matid[g] !== -1) {
      const matId = model.geom_matid[g];
      color = [
        model.mat_rgba[matId * 4 + 0],
        model.mat_rgba[matId * 4 + 1],
        model.mat_rgba[matId * 4 + 2],
        model.mat_rgba[matId * 4 + 3],
      ];
    } else {
      color = BLACK_MESH_NAME.test(partName) ? FALLBACK_BLACK : FALLBACK_SILVER;
    }

    const isFloor = type === mujoco.mjtGeom.mjGEOM_PLANE.value;
    // No environment map here (deliberately -- it made blacks read as grey,
    // or worse, blew the whole model out toward white), so a part's color
    // has to come from the scene's actual lights, not reflection. Keep
    // metalness low enough that the diffuse response (color x light) still
    // dominates -- black stays black, silver stays silver -- and get gloss
    // from a fairly low roughness (a tight specular highlight) instead.
    const material = isFloor
      ? new THREE.MeshStandardMaterial({
          map: getGridFloorTexture(planeWidth, planeHeight),
          roughness: 0.9,
          metalness: 0.05,
        })
      : new THREE.MeshStandardMaterial({
          color: new THREE.Color(color[0], color[1], color[2]),
          transparent: color[3] < 1.0,
          opacity: color[3],
          roughness: 0.35,
          metalness: 0.25,
        });

    const mesh = new THREE.Mesh(geometry, material);
    mesh.castShadow = type !== mujoco.mjtGeom.mjGEOM_PLANE.value;
    mesh.receiveShadow = true;
    (mesh as unknown as { bodyID: number }).bodyID = b;
    bodies[b].add(mesh);
    getPosition(model.geom_pos, g, mesh.position);
    if (type !== mujoco.mjtGeom.mjGEOM_PLANE.value) getQuaternion(model.geom_quat, g, mesh.quaternion);
  }

  for (let l = 0; l < model.nlight; l++) {
    const light = new THREE.DirectionalLight(0xffffff, 1.5);
    root.add(light);
    lights.push(light);
  }
  if (model.nlight === 0) {
    root.add(new THREE.DirectionalLight(0xffffff, 1.5));
  }

  for (let b = 0; b < model.nbody; b++) {
    if (b === 0 || !bodies[0]) {
      root.add(bodies[b] ?? new THREE.Group());
    } else if (bodies[b]) {
      bodies[0].add(bodies[b]);
    }
  }

  return { root, bodies, lights };
}
