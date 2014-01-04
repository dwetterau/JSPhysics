function ForceGenerator() {
  this.bodyIds = {};
}

ForceGenerator.prototype.updateForce = function(body, dt) {
  throw "Unimplemented updateForce method!";
};

ForceGenerator.prototype.addBodyId = function(id) {
  this.bodyIds[id] = id;
};

ForceGenerator.prototype.removeBodyId = function(id) {
  delete this.bodyIds[id];
};

ForceGenerator.prototype.getBodyIds = function() {
  return this.bodyIds;
};

function Gravity(g) {
  this.gravity = g;
}

// Gravity inherits from ForceGenerator
Gravity.prototype = new ForceGenerator();
Gravity.prototype.constructor = Gravity;

Gravity.prototype.updateForce = function(body, dt) {
  if (!body.hasFiniteMass()) {
    return;
  }
  body.addForce(this.gravity.scale(body.getMass()));
};