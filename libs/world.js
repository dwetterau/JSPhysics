function World() {
  this.objectIds = [];
  this.objects = {};
  this.objectCounter = 0;
  this.forceGenerators = [];
}

World.prototype.addObject = function(body) {
  this.objectCounter += 1;
  this.objectIds.push(this.objectCounter);
  this.objects[this.objectCounter] = body;
};

World.prototype.updateAllForces = function(dt) {
  for(var i = 0; i < this.forceGenerators.length; i++) {
    var objectsToApply = this.forceGenerators[i].getObjectIds();
    for (var j = 0; j < objectsToApply.length; j++) {
      this.forceGenerators[i].updateForce(this.objects[objectsToApply[j]], dt);
    }
  }
};

World.prototype.runPhysics = function(dt) {
  this.updateAllForces(dt);
  for (var i = 0; i < this.objectIds.length; i++) {
    this.objects[this.objectIds[i]].integrate(dt);
  }
};

