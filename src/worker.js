import {
  World,
  NaiveBroadphase,
  SAPBroadphase,
  Body,
  Plane,
  Box,
  Vec3,
  ConvexPolyhedron,
  Cylinder,
  Heightfield,
  Particle,
  Sphere,
  Trimesh,
  PointToPointConstraint,
  ConeTwistConstraint,
  HingeConstraint,
  DistanceConstraint,
  LockConstraint,
  Constraint,
  Spring,
  Material,
  Quaternion,
  Ray,
  RaycastResult,
} from 'cannon-es'

export default class Worker {
  constructor() {
    this.bodies = {}
    this.springs = {}
    this.rays = {}
    this.world = new World()
    this.config = { step: 1 / 60 }
    this.subscriptions = {}
    this.tempVector = new Vec3()
    this.listeners = []
  }

  createShape(type, args) {
    switch (type) {
      case 'Box':
        return new Box(new Vec3(...args)) // halfExtents
      case 'ConvexPolyhedron':
        const [v, f, n] = args
        return new ConvexPolyhedron({
          vertices: v.map(([x, y, z]) => new Vec3(x, y, z)),
          normals: n ? n.map(([x, y, z]) => new Vec3(x, y, z)) : null,
          faces: f,
        })
      case 'Cylinder':
        return new Cylinder(...args) // [ radiusTop, radiusBottom, height, numSegments ] = args
      case 'Heightfield':
        return new Heightfield(...args) // [ Array data, options: {minValue, maxValue, elementSize}  ] = args
      case 'Particle':
        return new Particle() // no args
      case 'Plane':
        return new Plane() // no args, infinite x and y
      case 'Sphere':
        return new Sphere(...args) // [radius] = args
      case 'Trimesh':
        return new Trimesh(...args) // [vertices, indices] = args
    }
  }

  syncBodies() {
    this.postMessage({ op: 'sync', bodies: this.world.bodies.map((body) => body.uuid) })
    this.bodies = this.world.bodies.reduce((acc, body) => ({ ...acc, [body.uuid]: body }), {})
  }

  postMessage(e) {
    this.onmessage({ data: e })
  }

  addListener(fn) {
    this.listeners.push(fn)
  }

  onmessage(e) {
    const { op, uuid, type, positions, quaternions, props } = e.data

    switch (op) {
      case 'init': {
        const {
          gravity,
          tolerance,
          step,
          iterations,
          allowSleep,
          broadphase,
          axisIndex,
          defaultContactMaterial,
        } = props
        const broadphases = { NaiveBroadphase, SAPBroadphase }
        this.world.allowSleep = allowSleep
        this.world.gravity.set(gravity[0], gravity[1], gravity[2])
        this.world.solver.tolerance = tolerance
        this.world.solver.iterations = iterations
        this.world.broadphase = new (broadphases[broadphase + 'Broadphase'] || NaiveBroadphase)(this.world)
        this.world.broadphase.axisIndex = axisIndex ?? 0
        Object.assign(this.world.defaultContactMaterial, defaultContactMaterial)
        this.config.step = step
        break
      }
      case 'step': {
        this.world.step(this.config.step)
        const numberOfBodies = this.world.bodies.length
        for (let i = 0; i < numberOfBodies; i++) {
          let b = this.world.bodies[i],
            p = b.position,
            q = b.quaternion
          positions[3 * i + 0] = p.x
          positions[3 * i + 1] = p.y
          positions[3 * i + 2] = p.z
          quaternions[4 * i + 0] = q.x
          quaternions[4 * i + 1] = q.y
          quaternions[4 * i + 2] = q.z
          quaternions[4 * i + 3] = q.w
        }
        const observations = []
        for (const id of Object.keys(this.subscriptions)) {
          const [uuid, type] = this.subscriptions[id]
          let value = this.bodies[uuid][type]
          if (value instanceof Vec3) value = value.toArray()
          else if (value instanceof Quaternion) {
            value.toEuler(this.tempVector)
            value = this.tempVector.toArray()
          }
          observations.push([id, value])
        }
        this.postMessage({
          op: 'frame',
          positions,
          quaternions,
          observations,
          active: this.world.hasActiveBodies,
        })
        break
      }
      case 'addBodies': {
        for (let i = 0; i < uuid.length; i++) {
          const {
            args = [],
            position = [0, 0, 0],
            rotation = [0, 0, 0],
            scale = [1, 1, 1],
            type: bodyType,
            mass,
            material,
            shapes,
            onCollide,
            ...extra
          } = props[i]

          const body = new Body({
            ...extra,
            mass: bodyType === 'Static' ? 0 : mass,
            type: bodyType ? Body[bodyType.toUpperCase()] : undefined,
            material: material ? new Material(material) : undefined,
          })
          body.uuid = uuid[i]

          if (type === 'Compound') {
            shapes.forEach(({ type, args, position, rotation, material, ...extra }) => {
              const shapeBody = body.addShape(
                this.createShape(type, args),
                position ? new Vec3(...position) : undefined,
                rotation ? new Quaternion().setFromEuler(...rotation) : undefined
              )
              if (material) shapeBody.material = new Material(material)
              Object.assign(shapeBody, extra)
            })
          } else {
            body.addShape(this.createShape(type, args))
          }

          body.position.set(position[0], position[1], position[2])
          body.quaternion.setFromEuler(rotation[0], rotation[1], rotation[2])
          this.world.addBody(body)

          if (onCollide)
            body.addEventListener('collide', ({ type, contact, target }) => {
              const { ni, ri, rj } = contact
              this.postMessage({
                op: 'event',
                type,
                body: body.uuid,
                target: target.uuid,
                contact: {
                  ni: ni.toArray(),
                  ri: ri.toArray(),
                  rj: rj.toArray(),
                  impactVelocity: contact.getImpactVelocityAlongNormal(),
                },
                collisionFilters: {
                  bodyFilterGroup: body.collisionFilterGroup,
                  bodyFilterMask: body.collisionFilterMask,
                  targetFilterGroup: target.collisionFilterGroup,
                  targetFilterMask: target.collisionFilterMask,
                },
              })
            })
        }
        this.syncBodies()
        break
      }
      case 'removeBodies': {
        for (let i = 0; i < uuid.length; i++) this.world.removeBody(this.bodies[uuid[i]])
        this.syncBodies()
        break
      }
      case 'subscribe': {
        const { id, type } = props
        this.subscriptions[id] = [uuid, type]
        break
      }
      case 'unsubscribe': {
        delete this.subscriptions[props]
        break
      }
      case 'setPosition':
        this.bodies[uuid].position.set(props[0], props[1], props[2])
        break
      case 'setQuaternion':
        this.bodies[uuid].quaternion.setFromEuler(props[0], props[1], props[2])
        break
      case 'setVelocity':
        this.bodies[uuid].velocity.set(props[0], props[1], props[2])
        break
      case 'setAngularVelocity':
        this.bodies[uuid].angularVelocity.set(props[0], props[1], props[2])
        break
      case 'setMass':
        this.bodies[uuid].mass = props
        break
      case 'setLinearDamping':
        this.bodies[uuid].linearDamping = props
        break
      case 'setAngularDamping':
        this.bodies[uuid].angularDamping = props
        break
      case 'setAllowSleep':
        this.bodies[uuid].allowSleep = props
        break
      case 'setSleepSpeedLimit':
        this.bodies[uuid].sleepSpeedLimit = props
        break
      case 'setSleepTimeLimit':
        this.bodies[uuid].sleepTimeLimit = props
        break
      case 'setCollisionFilterGroup':
        this.bodies[uuid].collisionFilterGroup = props
        break
      case 'setCollisionFilterMask':
        this.bodies[uuid].collisionFilterMask = props
        break
      case 'setCollisionFilterMask':
        this.bodies[uuid].collisionFilterMask = props
        break
      case 'setFixedRotation':
        this.bodies[uuid].fixedRotation = props
        break
      case 'applyForce':
        this.bodies[uuid].applyForce(new Vec3(...props[0]), new Vec3(...props[1]))
        break
      case 'applyImpulse':
        this.bodies[uuid].applyImpulse(new Vec3(...props[0]), new Vec3(...props[1]))
        break
      case 'applyLocalForce':
        this.bodies[uuid].applyLocalForce(new Vec3(...props[0]), new Vec3(...props[1]))
        break
      case 'applyLocalImpulse':
        this.bodies[uuid].applyLocalImpulse(new Vec3(...props[0]), new Vec3(...props[1]))
        break
      case 'addConstraint': {
        const [bodyA, bodyB, optns] = props
        let { pivotA, pivotB, axisA, axisB, ...options } = optns

        // is there a better way to enforce defaults?
        pivotA = Array.isArray(pivotA) ? new Vec3(...pivotA) : undefined
        pivotB = Array.isArray(pivotB) ? new Vec3(...pivotB) : undefined
        axisA = Array.isArray(axisA) ? new Vec3(...axisA) : undefined
        axisB = Array.isArray(axisB) ? new Vec3(...axisB) : undefined

        let constraint

        switch (type) {
          case 'PointToPoint':
            constraint = new PointToPointConstraint(
              this.bodies[bodyA],
              pivotA,
              this.bodies[bodyB],
              pivotB,
              optns.maxForce
            )
            break
          case 'ConeTwist':
            constraint = new ConeTwistConstraint(this.bodies[bodyA], this.bodies[bodyB], {
              pivotA,
              pivotB,
              axisA,
              axisB,
              ...options,
            })
            break
          case 'Hinge':
            constraint = new HingeConstraint(this.bodies[bodyA], this.bodies[bodyB], {
              pivotA,
              pivotB,
              axisA,
              axisB,
              ...options,
            })
            break
          case 'Distance':
            constraint = new DistanceConstraint(
              this.bodies[bodyA],
              this.bodies[bodyB],
              optns.distance,
              optns.maxForce
            )
            break
          case 'Lock':
            constraint = new LockConstraint(this.bodies[bodyA], this.bodies[bodyB], optns)
            break
          default:
            constraint = new Constraint(this.bodies[bodyA], this.bodies[bodyB], optns)
            break
        }
        constraint.uuid = uuid
        this.world.addConstraint(constraint)
        break
      }
      case 'removeConstraint':
        this.world.removeConstraint(uuid)
        break

      case 'enableConstraint':
        this.world.constraints.filter(({ uuid: thisId }) => thisId === uuid).map((c) => c.enable())
        break

      case 'disableConstraint':
        this.world.constraints.filter(({ uuid: thisId }) => thisId === uuid).map((c) => c.disable())
        break

      case 'addSpring': {
        const [bodyA, bodyB, optns] = props
        let { worldAnchorA, worldAnchorB, localAnchorA, localAnchorB, restLength, stiffness, damping } = optns

        worldAnchorA = Array.isArray(worldAnchorA) ? new Vec3(...worldAnchorA) : undefined
        worldAnchorB = Array.isArray(worldAnchorB) ? new Vec3(...worldAnchorB) : undefined
        localAnchorA = Array.isArray(localAnchorA) ? new Vec3(...localAnchorA) : undefined
        localAnchorB = Array.isArray(localAnchorB) ? new Vec3(...localAnchorB) : undefined

        let spring = new Spring(this.bodies[bodyA], this.bodies[bodyB], {
          worldAnchorA,
          worldAnchorB,
          localAnchorA,
          localAnchorB,
          restLength,
          stiffness,
          damping,
        })

        spring.uuid = uuid

        let postStepSpring = (e) => spring.applyForce()

        this.springs[uuid] = postStepSpring

        // Compute the force after each step
        this.world.addEventListener('postStep', this.springs[uuid])
        break
      }
      case 'removeSpring': {
        this.world.removeEventListener('postStep', this.springs[uuid])
        break
      }
      case 'addRay': {
        const { from, to, ...options } = props
        const ray = new Ray(from ? new Vec3(...from) : undefined, to ? new Vec3(...to) : undefined)
        options.mode = Ray[options.mode.toUpperCase()]
        options.result = new RaycastResult()
        this.rays[uuid] = () => {
          ray.intersectWorld(this.world, options)
          const {
            body,
            shape,
            rayFromWorld,
            rayToWorld,
            hitNormalWorld,
            hitPointWorld,
            ...rest
          } = options.result
          this.postMessage({
            op: 'event',
            type: 'rayhit',
            ray: {
              from,
              to,
              direction: ray.direction.toArray(),
              collisionFilterGroup: ray.collisionFilterGroup,
              collisionFilterMask: ray.collisionFilterMask,
              uuid,
            },
            body: body ? body.uuid : null,
            shape: shape ? { ...shape, body: body.uuid } : null,
            rayFromWorld: rayFromWorld.toArray(),
            rayToWorld: rayToWorld.toArray(),
            hitNormalWorld: hitNormalWorld.toArray(),
            hitPointWorld: hitPointWorld.toArray(),
            ...rest,
          })
        }
        this.world.addEventListener('preStep', this.rays[uuid])
        break
      }
      case 'removeRay': {
        this.world.removeEventListener('preStep', this.rays[uuid])
        delete this.rays[uuid]
        break
      }
    }

    this.listeners.forEach((fn) => fn(e))
  }

  terminate() {
    console.log('TERMINATE')
  }
}
