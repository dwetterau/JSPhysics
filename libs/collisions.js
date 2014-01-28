function Collision(point, normal, penetration) {
  this.point = point;
  this.normal = normal;
  this.penetration = penetration;
  this.contactToWorld = new Matrix3(new Array(9));
}

Collision.prototype.angularLimit = .2;

Collision.prototype.setBodyIds = function(firstId, secondId) {
  this.firstId = firstId;
  this.secondId = secondId;
  this.ids = [firstId, secondId];
};

Collision.prototype.updatePosition = function() {
  this.calculateInertia();
  return this.applyPositionChange();
};

Collision.prototype.updateVelocity = function() {
  this.calculateImpulseContact();
  return this.applyVelocityChange();
};

Collision.prototype.calculateRelativePositions = function() {
  var relativePosition = {};
  relativePosition[this.firstId] = this.point.sub(this.worldBodies[this.firstId].position);
  relativePosition[this.secondId] = this.point.sub(this.worldBodies[this.secondId].position);
  this.relativePosition = relativePosition;
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

Collision.prototype.calculateInitialContactVelocity = function() {
  var contactVelocity = this.calculateLocalVelocity(this.worldBodies[this.firstId]);
  contactVelocity.subInPlace(this.calculateLocalVelocity(this.worldBodies[this.secondId]));
  this.contactVelocity = contactVelocity;
};

Collision.prototype.calculateLocalVelocity = function(body) {
  var velocity = body.getRotation().cross(this.point.sub(body.position));
  velocity.addInPlace(body.getVelocity());
  return this.contactToWorld.multiplyVectorTranspose(velocity);
};

Collision.prototype.calculateDesiredVelocity = function() {
  this.desiredDeltaVelocity = -this.contactVelocity.x * (1 + this.restitution);
};

Collision.prototype.calculateImpulseContact = function() {
  var body1 = this.worldBodies[this.firstId];
  var body2 = this.worldBodies[this.secondId];

  var deltaVelocity = 0, deltaVWorld;
  if (body1.hasFiniteMass()) {
    deltaVWorld = this.relativePosition[this.firstId].cross(this.normal);
    deltaVWorld = body1.inverseInertiaTensor.multiplyVector(deltaVWorld);
    deltaVWorld = deltaVWorld.cross(this.relativePosition[this.firstId]);
    deltaVelocity += body1.getInverseMass() + deltaVWorld.dot(this.normal);
  }
  if (body2.hasFiniteMass()) {
    deltaVWorld = this.relativePosition[this.secondId].cross(this.normal);
    deltaVWorld = body2.inverseInertiaTensor.multiplyVector(deltaVWorld);
    deltaVWorld = deltaVWorld.cross(this.relativePosition[this.secondId]);
    deltaVelocity += body2.getInverseMass() + deltaVWorld.dot(this.normal);
  }
  this.impulseContact = new Vector3(this.desiredDeltaVelocity / deltaVelocity, 0, 0);
};

Collision.prototype.applyVelocityChange = function() {
  var impulse = this.contactToWorld.multiplyVector(this.impulseContact);
  var body1 = this.worldBodies[this.firstId];
  var body2 = this.worldBodies[this.secondId];

  var impulsiveTorque, velocityChange, rotationChange;
  var changes = {};
  if (body1.hasFiniteMass()) {
    impulsiveTorque = this.relativePosition[this.firstId].cross(impulse);
    rotationChange = body1.inverseInertiaTensor.multiplyVector(impulsiveTorque);
    velocityChange = new Vector3(0, 0, 0);
    velocityChange.addScaledVector(impulse, body1.getInverseMass());

    body1.addVelocity(velocityChange);
    body1.addRotation(rotationChange);
    changes[this.firstId] = {
      velocityChange: velocityChange,
      rotationChange: rotationChange
    }
  }

  if (body2.hasFiniteMass()) {
    impulsiveTorque = this.relativePosition[this.secondId].cross(impulse);
    rotationChange = body2.inverseInertiaTensor.multiplyVector(impulsiveTorque).scale(-1);
    velocityChange = new Vector3(0, 0, 0);
    velocityChange.addScaledVector(impulse, -body2.getInverseMass());

    body2.addVelocity(velocityChange);
    body2.addRotation(rotationChange);
    changes[this.secondId] = {
      velocityChange: velocityChange,
      rotationChange: rotationChange
    }
  }
  return changes;
};

Collision.prototype.calculateInertia = function() {
  var body1 = this.worldBodies[this.firstId];
  var body2 = this.worldBodies[this.secondId];

  var angularInertiaWorld, angularInertia = 0, linearInertia = 0, totalInertia = 0;
  if (body1.hasFiniteMass()) {
    angularInertiaWorld = this.relativePosition[this.firstId].cross(this.normal);
    angularInertiaWorld = body1.inverseInertiaTensorWorld.multiplyVector(angularInertiaWorld);
    angularInertiaWorld = angularInertiaWorld.cross(this.relativePosition[this.firstId]);
    angularInertia = angularInertiaWorld.dot(this.normal);
    linearInertia = body1.getInverseMass();
    totalInertia += angularInertia + linearInertia;
  }
  this.body1Inertia = {
    angular: angularInertia,
    linear: linearInertia
  };
  angularInertia = 0;
  linearInertia = 0;

  if (body2.hasFiniteMass()) {
    angularInertiaWorld = this.relativePosition[this.secondId].cross(this.normal);
    angularInertiaWorld = body2.inverseInertiaTensorWorld.multiplyVector(angularInertiaWorld);
    angularInertiaWorld = angularInertiaWorld.cross(this.relativePosition[this.secondId]);
    angularInertia = angularInertiaWorld.dot(this.normal);
    linearInertia = body2.getInverseMass();
    totalInertia += angularInertia + linearInertia;
  }
  this.body2Inertia = {
    angular: angularInertia,
    linear: linearInertia
  };
  this.body1Move = this.correctMovements(
      body1, 1, this.body1Inertia.linear, this.body1Inertia.angular, totalInertia);
  this.body2Move = this.correctMovements(
      body2, -1, this.body2Inertia.linear, this.body2Inertia.angular, totalInertia);
};

Collision.prototype.correctMovements = function(body, sign, linearInertia, angularInertia, totalInertia) {
  var angularMove = sign * this.penetration * (angularInertia / totalInertia);
  var linearMove = sign * this.penetration * (linearInertia / totalInertia);

  var projection = this.point.sub(body.position);
  projection.addScaledVector(this.normal, -projection.dot(this.normal));
  var maxMagnitude = this.angularLimit * projection.magnitude();
  var totalMove;
  if (angularMove < -maxMagnitude) {
    totalMove = angularMove + linearMove;
    angularMove = -maxMagnitude;
    linearMove = totalMove - angularMove;
  } else if (angularMove > maxMagnitude) {
    totalMove = angularMove + linearMove;
    angularMove = maxMagnitude;
    linearMove = totalMove - angularMove;
  }
  return {
    linear: linearMove,
    angular: angularMove
  };
};

Collision.prototype.applyPositionChange = function() {
  var body1 = this.worldBodies[this.firstId];
  var body2 = this.worldBodies[this.secondId];
  var movements = {};

  if (body1.hasFiniteMass()) {
    movements[this.firstId] = this.applyPositionChangeToBody(body1, this.body1Move, this.body1Inertia);
  }
  if (body2.hasFiniteMass()) {
    movements[this.secondId] = this.applyPositionChangeToBody(body2, this.body2Move, this.body2Inertia);
  }
  return movements;
};

Collision.prototype.applyPositionChangeToBody = function(body, movements, inertia) {
  var angularChange;
  if (movements.angular == 0) {
    angularChange = new Vector3(0, 0, 0);
  } else {
    var targetAngularDirection = this.point.sub(body.position).cross(this.normal);
    angularChange = body.inverseInertiaTensorWorld.multiplyVector(targetAngularDirection)
        .scale(movements.angular / inertia.angular);
  }
  var linearChange = this.normal.scale(movements.linear);
  body.addMovement(linearChange);
  var q = body.getOrientation();
  q.addScaledVector(angularChange, 1.0);
  body.setOrientation(q);
  // TODO: something with awakening and seeing if I need this update
  body.calculateDerivedData();
  return {
    linearChange: linearChange,
    angularChange: angularChange
  };
};

function CollisionResolver(threshold, maxIterations) {
  this.threshold = threshold;
  this.maxIterations = maxIterations;
}

CollisionResolver.prototype.initializeContacts = function(contacts, worldBodies, restitution, dt) {
  contacts.forEach(function(contact) {
    contact.worldBodies = worldBodies;
    contact.restitution = restitution;
    contact.dt = dt;
    contact.calculateRelativePositions();
    contact.calculateContactBasis();
    contact.calculateInitialContactVelocity();
    contact.calculateDesiredVelocity();
  });
};

CollisionResolver.prototype.adjustPositions = function(contacts, worldBodies) {
  var iterationNumber = 0;
  while (iterationNumber < this.maxIterations) {
    var maxPenetration = this.threshold;
    var index = -1, i = 0;
    for (i = 0; i < contacts.length; i++) {
      if (contacts[i].penetration > maxPenetration) {
        maxPenetration = contacts[i].penetration;
        index = i;
      }
    }
    if (index == -1) {
      // All collisions resolved!
      break;
    }
    // TODO: something with being awake
    var movements = contacts[index].updatePosition();
    // Adjust possibly all contacts after updating the position
    contacts.forEach(function(contact) {
      contact.ids.forEach(function(bodyId) {
        if (!worldBodies[bodyId].hasFiniteMass()) {
          // Can't move things with infinite mass anyway...
          return;
        }
        contacts[index].ids.forEach(function(movedBodyId) {
          if (bodyId == movedBodyId) {
            var deltaPosition = movements[movedBodyId].linearChange.add(
                movements[movedBodyId].angularChange.cross(contact.relativePosition[bodyId]));
            var sign = (bodyId == contact.secondId) ? 1 : -1;
            contact.penetration += sign * deltaPosition.dot(contact.normal);
          }
        });
      });
    });
    iterationNumber++;
  }
};

CollisionResolver.prototype.adjustVelocities = function(contacts, worldBodies) {
  var iterationNumber = 0;
  while (iterationNumber < this.maxIterations) {
    var maxVelocity = this.threshold;
    var index = -1, i = 0;
    for (i = 0; i < contacts.length; i++) {
      if (contacts[i].desiredDeltaVelocity > maxVelocity) {
        maxVelocity = contacts[i].desiredDeltaVelocity;
        index = i;
      }
    }
    if (index == -1) {
      // All velocities resolved!
      break;
    }
    // TODO: something with being awake
    var velocities = contacts[index].updateVelocity();
    // Adjust possibly all contacts after updating the position
    contacts.forEach(function(contact) {
      contact.ids.forEach(function(bodyId) {
        if (!worldBodies[bodyId].hasFiniteMass()) {
          // Can't move things with infinite mass anyway...
          return;
        }
        contacts[index].ids.forEach(function(movedBodyId) {
          if (bodyId == movedBodyId) {
            var deltaVelocity = velocities[movedBodyId].velocityChange.add(
                velocities[movedBodyId].rotationChange.cross(contact.relativePosition[bodyId])
            );
            var sign = (bodyId == contact.secondId) ? -1 : 1;
            contact.contactVelocity.addInPlace(
                contact.contactToWorld.multiplyVectorTranspose(deltaVelocity).scale(sign));
            contact.calculateDesiredVelocity();
          }
        });
      });
    });
    iterationNumber++;
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
  if (body_1.isBox() && body_2.isPlane()) {
    return this.getBoxPlaneCollisions(body_1, body_2, false);
  }
  if (body_1.isPlane() && body_2.isBox()) {
    return this.getBoxPlaneCollisions(body_2, body_1, true);
  }
  if (body_1.isPlane() && body_2.isPlane()) {
    return [];
  }
  if (body_1.isBox() && body_2.isSphere()) {
    return this.getBoxSphereCollisions(body_1, body_2, false);
  }
  if (body_1.isSphere() && body_2.isBox()) {
    return this.getBoxSphereCollisions(body_2, body_1, true);
  }
  if (body_1.isBox() && body_2.isBox()) {
    return this.getBoxBoxCollisions(body_1, body_2);
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
          (flipped) ? plane.getGeometry().normal.scale(-1) : plane.getGeometry().normal.copy(),
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
    ];
  }
};

CollisionDetector.prototype.getBoxPlaneCollisions = function(box, plane, flipped) {
  var coefficients = [[1, 1, 1], [-1, -1, -1], [-1, 1, 1], [1, -1, 1],
                [1, 1, -1], [-1, -1, 1], [-1, 1, -1], [1, -1, -1]];
  var i;
  var collisions = [];
  for (i = 0; i < 8; i++) {
    var point = new Vector3(coefficients[i][0] * box.getGeometry().dx_h,
        coefficients[i][1] * box.getGeometry().dy_h,
        coefficients[i][2] * box.getGeometry().dz_h);
    point = box.getPointInWorldSpace(point);
    var distance = plane.getGeometry().normal.dot(point);
    if (distance <= plane.getGeometry().offset + this.threshold) {
      collisions.push(new Collision(
        plane.getGeometry().normal.scale(plane.getGeometry().offset - distance).add(point),
        (flipped ? plane.getGeometry().normal.scale(-1) : plane.getGeometry().normal.copy()),
        plane.getGeometry().offset - distance
      ));
    }
  }
  return collisions;
};

CollisionDetector.prototype.getBoxSphereCollisions = function(box, sphere, flipped) {
  var relativeCenter = box.getPointInBodySpace(sphere.getPosition());
  var r = sphere.getGeometry().r;
  var dx_h = box.getGeometry().dx_h;
  var dy_h = box.getGeometry().dy_h;
  var dz_h = box.getGeometry().dz_h;
  debugger;
  if (Math.abs(relativeCenter.x) - r > dx_h ||
      Math.abs(relativeCenter.y) - r > dy_h ||
      Math.abs(relativeCenter.z) - r > dz_h) {
    return [];
  }
  var closestPoint = new Vector3(0, 0, 0);
  var distance = relativeCenter.x;
  if (distance > dx_h) {
    distance = dx_h;
  } else if (distance < -dx_h) {
    distance = -dx_h
  }
  closestPoint.x = distance;

  distance = relativeCenter.y;
  if (distance > dy_h) {
    distance = dy_h;
  } else if (distance < -dy_h) {
    distance = -dy_h
  }
  closestPoint.y = distance;

  distance = relativeCenter.z;
  if (distance > dz_h) {
    distance = dz_h;
  } else if (distance < -dz_h) {
    distance = -dz_h
  }
  closestPoint.z = distance;

  distance = (closestPoint.sub(relativeCenter)).magnitudeSq();
  if (distance > r * r) {
    return [];
  }

  var closestPointWorld = box.getPointInWorldSpace(closestPoint);
  var normal = closestPointWorld.sub(sphere.getPosition());
  normal.normalize();
  if (flipped) {
    normal.scaleInPlace(-1);
  }
  return [new Collision(
      closestPointWorld,
      normal,
      r - Math.sqrt(distance)
  )];
};

CollisionDetector.prototype.getBoxBoxCollisions = function(box1, box2) {
  var toCenter = box2.getPosition().sub(box1.getPosition());
  var resultObj = {
    smallestPenetration: Number.MAX_VALUE,
    smallestCase: -1
  };
  //Check all 15 axes
  // Box1 axes
  resultObj = this.tryAxis(box1, box2, box1.getAxis(0), toCenter, 0,
      resultObj.smallestPenetration, resultObj.smallestCase);
  if (!resultObj.result) return [];
  resultObj = this.tryAxis(box1, box2, box1.getAxis(1), toCenter, 1,
    resultObj.smallestPenetration, resultObj.smallestCase);
  if (!resultObj.result) return [];
  resultObj = this.tryAxis(box1, box2, box1.getAxis(2), toCenter, 2,
    resultObj.smallestPenetration, resultObj.smallestCase);
  if (!resultObj.result) return [];

  // Box2 axes
  resultObj = this.tryAxis(box1, box2, box2.getAxis(0), toCenter, 3,
    resultObj.smallestPenetration, resultObj.smallestCase);
  if (!resultObj.result) return [];
  resultObj = this.tryAxis(box1, box2, box2.getAxis(1), toCenter, 4,
    resultObj.smallestPenetration, resultObj.smallestCase);
  if (!resultObj.result) return [];
  resultObj = this.tryAxis(box1, box2, box2.getAxis(2), toCenter, 5,
    resultObj.smallestPenetration, resultObj.smallestCase);
  if (!resultObj.result) return [];

  var bestSingleAxis = resultObj.smallestCase;
  // edge cross products
  resultObj = this.tryAxis(box1, box2, box1.getAxis(0).cross(box2.getAxis(0)), toCenter, 6,
    resultObj.smallestPenetration, resultObj.smallestCase);
  if (!resultObj.result) return [];
  resultObj = this.tryAxis(box1, box2, box1.getAxis(0).cross(box2.getAxis(1)), toCenter, 7,
    resultObj.smallestPenetration, resultObj.smallestCase);
  if (!resultObj.result) return [];
  resultObj = this.tryAxis(box1, box2, box1.getAxis(0).cross(box2.getAxis(2)), toCenter, 8,
    resultObj.smallestPenetration, resultObj.smallestCase);
  if (!resultObj.result) return [];
  resultObj = this.tryAxis(box1, box2, box1.getAxis(1).cross(box2.getAxis(0)), toCenter, 9,
    resultObj.smallestPenetration, resultObj.smallestCase);
  if (!resultObj.result) return [];
  resultObj = this.tryAxis(box1, box2, box1.getAxis(1).cross(box2.getAxis(1)), toCenter, 10,
    resultObj.smallestPenetration, resultObj.smallestCase);
  if (!resultObj.result) return [];
  resultObj = this.tryAxis(box1, box2, box1.getAxis(1).cross(box2.getAxis(2)), toCenter, 11,
    resultObj.smallestPenetration, resultObj.smallestCase);
  if (!resultObj.result) return [];
  resultObj = this.tryAxis(box1, box2, box1.getAxis(2).cross(box2.getAxis(0)), toCenter, 12,
    resultObj.smallestPenetration, resultObj.smallestCase);
  if (!resultObj.result) return [];
  resultObj = this.tryAxis(box1, box2, box1.getAxis(2).cross(box2.getAxis(1)), toCenter, 13,
    resultObj.smallestPenetration, resultObj.smallestCase);
  if (!resultObj.result) return [];
  resultObj = this.tryAxis(box1, box2, box1.getAxis(2).cross(box2.getAxis(2)), toCenter, 14,
    resultObj.smallestPenetration, resultObj.smallestCase);
  if (!resultObj.result) return [];

  if (resultObj.smallestCase == -1) {
    return [];
  }

  if (resultObj.smallestCase < 3) {
    // Vertex of box 2 is on a face of box 1
    return this.getPointFaceBoxBox(box1, box2, toCenter,
        resultObj.smallestCase, resultObj.smallestPenetration);
  } else if (resultObj.smallestCase < 6) {
    // Vertex of box 1 is on a face of box 2
    return this.getPointFaceBoxBox(box2, box1, toCenter.scale(-1),
        resultObj.smallestCase - 3, resultObj.smallestPenetration);
  } else {
    //debugger;
    // Edge to edge contact
    var bestCase = resultObj.smallestCase - 6;

    // Determine which axes
    var box1AxisIndex = Math.floor(bestCase / 3);
    var box2AxisIndex = bestCase % 3;
    var box1Axis = box1.getAxis(box1AxisIndex);
    var box2Axis = box2.getAxis(box2AxisIndex);
    var axis = box1Axis.cross(box2Axis);
    axis.normalize();

    if (axis.dot(toCenter) > 0) {
      axis.scaleInPlace(-1);
    }
    // now we know the axis but not the edges
    var box1EdgePoint = box1.getGeometry().halfSize;
    var box2EdgePoint = box2.getGeometry().halfSize;
    for (var i = 0; i < 3; i++) {
      if (i == box1AxisIndex) {
        box1EdgePoint[i] = 0;
      } else if(box1.getAxis(i).dot(axis) > 0) {
        box1EdgePoint[i] = -box1EdgePoint[i]
      }
      if (i == box2AxisIndex) {
        box2EdgePoint[i] = 0;
      } else if(box2.getAxis(i).dot(axis) < 0) {
        box2EdgePoint[i] = -box2EdgePoint[i]
      }
    }
    box1EdgePoint = box1.getPointInWorldSpace(box1EdgePoint);
    box2EdgePoint = box2.getPointInWorldSpace(box2EdgePoint);

    // Need to calculate the point of closest approach for the two edges
    var contactPoint = this.getEdgeEdgeContactPoint(
      box1EdgePoint, box1Axis, box1.getGeometry().halfSize[box1AxisIndex],
      box2EdgePoint, box2Axis, box2.getGeometry().halfSize[box2AxisIndex],
      bestSingleAxis > 2
    );

    return [new Collision(
      contactPoint,
      axis,
      resultObj.smallestPenetration
    )]
  }
};

CollisionDetector.prototype.getEdgeEdgeContactPoint = function(
    box1Point, box1Direction, box1Size,
    box2Point, box2Direction, box2Size, useBox1) {
  var betweenPoints, center1, center2;
  var dotBetween1, dotBetween2, dotOneTwo, box1DirSq, box2DirSq;
  var denominator, muA, muB;

  box1DirSq = box1Direction.magnitudeSq();
  box2DirSq = box2Direction.magnitudeSq();
  dotOneTwo = box2Direction.dot(box1Direction);
  betweenPoints = box1Point.sub(box2Point);
  dotBetween1 = box1Direction.dot(betweenPoints);
  dotBetween2 = box2Direction.dot(betweenPoints);
  denominator = box1DirSq * box2DirSq - dotOneTwo * dotOneTwo;
  if (Math.abs(denominator) < 0.0001) {
    return useBox1 ? box1Point : box2Point;
  }
  muA = (dotOneTwo * dotBetween2 - box2DirSq * dotBetween1) / denominator;
  muB = (box1DirSq * dotBetween2 - dotOneTwo * dotBetween1) / denominator;
  if (muA > box1Size || muA < -box1Size || muB > box2Size || muB < -box2Size) {
    return useBox1 ? box1Point : box2Point;
  } else {
    center1 = box1Point.add(box1Direction.scale(muA));
    center2 = box2Point.add(box2Direction.scale(muB));
    return center1.scale(.5).add(center2.scale(.5));
  }
};

CollisionDetector.prototype.getPointFaceBoxBox = function(box1, box2, toCenter, bestCase, penetration) {
  var normal = box1.getAxis(bestCase);
  if (normal.dot(toCenter) > 0) {
    normal.scaleInPlace(-1);
  }
  // Figure out which vertex of box2 we are colliding with
  var contactPoint = box2.getGeometry().halfSize.copy();
  if (box2.getAxis(0).dot(normal) < 0) {
    contactPoint.x = -contactPoint.x;
  }
  if (box2.getAxis(1).dot(normal) < 0) {
    contactPoint.y = -contactPoint.y;
  }
  if (box2.getAxis(2).dot(normal) < 0) {
    contactPoint.z = -contactPoint.z;
  }
  return [new Collision(
    box2.getPointInWorldSpace(contactPoint),
    normal,
    penetration
  )];
};

CollisionDetector.prototype.tryAxis = function(box1, box2, axis, toCenter, index, smallestPenetration, smallestCase) {
  if (axis.magnitudeSq() < 0.0001) {
    return {
      result: true,
      smallestPenetration: smallestPenetration,
      smallestCase: smallestCase
    };
  }
  var penetration = this.penetrationOnAxis(box1, box2, axis, toCenter);
  if (penetration < 0) {
    return {
      result: false,
      smallestPenetration: smallestPenetration,
      smallestCase: smallestCase
    };
  }
  if (penetration < smallestPenetration) {
    smallestPenetration = penetration;
    smallestCase = index;
  }
  return {
    result: true,
    smallestPenetration: smallestPenetration,
    smallestCase: smallestCase
  };
};

CollisionDetector.prototype.penetrationOnAxis = function(box1, box2, axis, toCenter) {
  var project1 = this.transformToAxis(box1, axis);
  var project2 = this.transformToAxis(box2, axis);

  var distance = Math.abs(toCenter.dot(axis));
  return project1 + project2 - distance;
};

CollisionDetector.prototype.transformToAxis = function(box, axis) {
  return box.getGeometry().dx_h * Math.abs(axis.dot(box.getAxis(0))) +
    box.getGeometry().dy_h * Math.abs(axis.dot(box.getAxis(1))) +
    box.getGeometry().dz_h * Math.abs(axis.dot(box.getAxis(2)));
};