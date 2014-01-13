function Collision(point, normal, penetration) {
  this.point = point;
  this.normal = normal;
  this.penetration = penetration;
  this.contactToWorld = new Matrix3(new Array(9));
}

Collision.prototype.setBodyIds = function(firstId, secondId) {
  this.firstId = firstId;
  this.secondId = secondId;
};

Collision.prototype.resolve = function(worldBodies, restitution, dt) {
  this.worldBodies = worldBodies;
  this.restitution = restitution;
  // Will be needed with friction
  this.dt = dt;

  this.calculateContactBasis();
  this.calculateDesiredVelocity();
  this.calculateImpulseContact();
  this.applyVelocityChange();
  // TODO: resolve interpenetration
};

Collision.prototype.calculateContactBasis = function() {

  var s, firstNormal, secondNormal;
  if (Math.abs(this.normal.x) > Math.abs(this.normal.y)) {
    s = 1.0 / Math.sqrt(this.normal.z * this.normal.z + this.normal.x * this.normal.x);
    firstNormal = new Vector3(
      this.normal.z * s,
      0,
      -this.normal.x * s
    );
    secondNormal = new Vector3(
      this.normal.y * firstNormal.x,
      this.normal.z * firstNormal.x - this.normal.x * firstNormal.z,
      -this.normal.y * firstNormal.x
    );
  } else {
    s = 1.0 / Math.sqrt(this.normal.z * this.normal.z + this.normal.y * this.normal.y);
    firstNormal = new Vector3(
      0,
      -this.normal.z * s,
      this.normal.y * s
    );
    secondNormal = new Vector3(
      this.normal.y * firstNormal.z - this.normal.z * firstNormal.y,
      -this.normal.x * firstNormal.z,
      this.normal.x * firstNormal.y
    );
  }
  this.contactToWorld.setComponents(this.normal, firstNormal, secondNormal);
};

Collision.prototype.calculateDesiredVelocity = function() {
  var contactVelocity = this.calculateLocalVelocity(this.worldBodies[this.firstId]);
  contactVelocity.subInPlace(this.calculateLocalVelocity(this.worldBodies[this.secondId]));
  this.desiredDeltaVelocity = -contactVelocity.x * (1 + this.restitution);
};

Collision.prototype.calculateLocalVelocity = function(body) {
  var velocity = body.getRotation().cross(this.point.sub(body.position));
  velocity.addInPlace(body.getVelocity());
  return this.contactToWorld.multiplyVectorTranspose(velocity);
};

Collision.prototype.calculateImpulseContact = function() {
  var body1 = this.worldBodies[this.firstId];
  var body2 = this.worldBodies[this.secondId];

  var deltaVelocity = 0;
  var relativeContactPosition, deltaVWorld;
  if (body1.hasFiniteMass()) {
    relativeContactPosition = this.point.sub(body1.position);
    deltaVWorld = relativeContactPosition.cross(this.normal);
    deltaVWorld = body1.inverseInertiaTensor.multiplyVector(deltaVWorld);
    deltaVWorld = deltaVWorld.cross(relativeContactPosition);
    deltaVelocity += body1.getInverseMass() + deltaVWorld.dot(this.normal);
  }
  if (body2.hasFiniteMass()) {
    relativeContactPosition = this.point.sub(body2.position);
    deltaVWorld = relativeContactPosition.cross(this.normal);
    deltaVWorld = body2.inverseInertiaTensor.multiplyVector(deltaVWorld);
    deltaVWorld = deltaVWorld.cross(relativeContactPosition);
    deltaVelocity += body2.getInverseMass() + deltaVWorld.dot(this.normal);
  }
  this.impulseContact = new Vector3(this.desiredDeltaVelocity / deltaVelocity, 0, 0);
};

Collision.prototype.applyVelocityChange = function() {
  var impulse = this.contactToWorld.multiplyVector(this.impulseContact);
  var body1 = this.worldBodies[this.firstId];
  var body2 = this.worldBodies[this.secondId];

  var impulsiveTorque, velocityChange, rotationChange;
  if (body1.hasFiniteMass()) {
    impulsiveTorque = this.point.sub(body1.position).cross(impulse);
    rotationChange = body1.inverseInertiaTensor.multiplyVector(impulsiveTorque);
    velocityChange = new Vector3(0, 0, 0);
    velocityChange.addScaledVector(impulse, body1.getInverseMass());

    body1.addVelocity(velocityChange);
    body1.addRotation(rotationChange);
  }

  if (body2.hasFiniteMass()) {
    impulsiveTorque = this.point.sub(body2.position).cross(impulse);
    rotationChange = body2.inverseInertiaTensor.multiplyVector(impulsiveTorque);
    velocityChange = new Vector3(0, 0, 0);
    velocityChange.addScaledVector(impulse, -body2.getInverseMass());

    body2.addVelocity(velocityChange);
    body2.addRotation(rotationChange);
  }
};

function CollisionDetector(threshold) {
  this.threshold = threshold;
}

CollisionDetector.prototype.getCollisions = function(body_1, body_2) {
  if (body_1.isSphere() && body_2.isSphere()) {
    return this.getSphereSphereCollisions(body_1, body_2);
  }
  if (body_1.isSphere() && body_2.isPlane()) {
    return this.getSpherePlaneCollisions(body_1, body_2, false);
  }
  if (body_1.isPlane() && body_2.isSphere()) {
    return this.getSpherePlaneCollisions(body_2, body_1, true);
  }
  throw Error("Unknown Geometry / Collision Type");
};

CollisionDetector.prototype.getSphereSphereCollisions = function(body_1, body_2) {
  var normalLine = body_1.getPosition().copy();
  normalLine.subInPlace(body_2.getPosition());

  var radius_1 = body_1.getGeometry().r;
  var radius_2 = body_2.getGeometry().r;

  var size = normalLine.magnitude();
  if (size <= this.threshold || size >= radius_1 + radius_2) {
    return [];
  }
  var normal = normalLine.scale(1.0 / size);
  return [
    new Collision(
        body_1.getPosition().add(normalLine.scale(.5)),
        normal,
        radius_1 + radius_2 - size
    )
  ];
};

CollisionDetector.prototype.getSpherePlaneCollisions = function(sphere, plane, flipped) {
  var centerDistance = plane.getGeometry().normal.dot(sphere.getPosition()) - plane.getGeometry().offset;

  if (plane.getGeometry().isHalfSpace) {
    var sphereDistance = centerDistance - sphere.getGeometry().r;
    if (sphereDistance >= this.threshold) {
      return [];
    }
    return [
      new Collision(
          sphere.getPosition().sub(plane.getGeometry().normal.scale(sphereDistance + sphere.getGeometry().r)),
          (flipped ? plane.getGeometry().normal.scale(-1) : plane.getGeometry().copy()),
          -sphereDistance
      )
    ];
  } else {
    var r = sphere.getGeometry().r;
    if (centerDistance * centerDistance > r * r) {
      return [];
    }
    var normal = plane.getGeometry().normal.copy();
    var penetration = -centerDistance;
    if (centerDistance < 0) {
      centerDistance = -centerDistance;
      normal.scaleInPlace(-1);
    }
    penetration += r;
    return [
      new Collision(
          sphere.getPosition().sub(plane.getGeometry().normal.scale(centerDistance)),
          (flipped ? normal.scale(-1) : normal),
          penetration
      )
    ]
  }
};