function Vector(d1, d2) {
    this.a = d1;
    this.b = d2;
}

makeVector = function(theta, mag) {
    return new Vector(mag*Math.cos(theta), mag*Math.sin(theta));
}

Vector.prototype.addVector = function(v2) {
    this.a += v2.a;
    this.b += v2.b;
}

Vector.prototype.dot = function(v2) {
    return this.a*v2.a + this.b*v2.b;
}

Vector.prototype.proj = function(v2) {
    fraction = this.dot(v2)/v2.dot(v2);
    return new Vector(v2.a*fraction, v2.b*fraction);
}

Vector.prototype.getMag = function() {
    return Math.sqrt(this.a*this.a + this.b*this.b);
}

Vector.prototype.normalize = function() {
    mag = this.getMag();
    this.a /= mag;
    this.b /= mag;
}

Vector.prototype.reverse = function() {
    this.a = -this.a;
    this.b = -this.b;
}

Vector.prototype.makeMag = function(length) {
    mag = this.getMag();
    this.a *= length/mag;
    this.b *= length/mag;
}

Vector.prototype.mult = function(n) {
    this.a *= n;
    this.b *= n;
}

function Ball(r, p, theta, mag, mass, color) {
    this.radius = r;
    this.velocity = makeVector(theta, mag);
    this.mass = mass;
    this.color = color;
    this.pos = p;
}

Ball.prototype.getMomentum = function() {
    mom = new Vector(this.velocity.a, this.velocity.b);
    mom.makeMag(mom.getMag()*mass);
    return mom;
}

function Point(x, y) {
    this.x = x;
    this.y = y;
}
