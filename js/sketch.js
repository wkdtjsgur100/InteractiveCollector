var boids = [];
var moneys = [];
var moneyPic;
var person;
var happyPerson;
function setup() {
    createCanvas(1400, 800);
    person = loadImage("../img/person.png");
    happyPerson = loadImage("../img/happyperson.png");
    moneyPic = loadImage("../img/50000s.png");
    // Add an initial set of boids into the system
    for (var i = 0; i < 5; i++) {
        boids[i] = new Boid(random(width), random(height));
    }
}

function draw() {
    background(51);
    // Run all the boids
    for (var i = 0; i < boids.length; i++) {
        boids[i].run(boids);
    }
    // Draw all moneys
    for (var i = 0; i < moneys.length; i++) {
        moneys[i].run(boids);
    }
    var newMoneys = [];
    for (var i = 0; i < moneys.length; i++) {
        if (moneys[i] != null) {
            newMoneys.push(moneys[i]);
        }
    }
    moneys = newMoneys;
}

function mousePressed() {
    moneys[moneys.length] = new Money(mouseX, mouseY);
}

// Money class
// Creates the money class with ust position
function Money(x, y) {
    this.position = createVector(x, y);
}

// Ran for each frame
Money.prototype.run = function (boids) {
    if (this.checkEaten(boids)) {
        var index = moneys.indexOf(this);
        moneys[index] = null;
    } else {
        this.render();
    }
}

// Check if the money should be deleted
Money.prototype.checkEaten = function (boids) {
    var eatingRadius = 75;
    var eaten = false;
    for (var i = 0; i < boids.length; i++) {
        var d = p5.Vector.dist(this.position, boids[i].position);
        if (d < eatingRadius) {
            eaten = true;
        }
    }
    return eaten;
}


// Render draw money
Money.prototype.render = function () {
    image(moneyPic, this.position.x, this.position.y, moneyPic.width/ 4, moneyPic.height/ 4);
}

// Boid class
// Methods for Separation, Cohesion, Alignment added
function Boid(x, y) {
    this.acceleration = createVector(0, 0);
    this.velocity = p5.Vector.random2D();
    this.position = createVector(x, y);
    this.r = 3.0;
    this.maxspeed = 3; // Maximum speed
    this.maxforce = 0.05; // Maximum steering force
}

Boid.prototype.run = function (boids) {
    this.flock(boids);
    this.update();
    this.borders();
    this.render();
}

// Forces go into acceleration
Boid.prototype.applyForce = function (force) {
    this.acceleration.add(force);
}

// We accumulate a new acceleration each time based on three rules
Boid.prototype.flock = function (boids) {
    var sep = this.separate(boids); // Separation
    var ali = this.align(boids); // Alignment
    var coh = this.cohesion(boids); // Cohesion
    // Arbitrarily weight these forces
    sep.mult(2.0);
    ali.mult(1.0);
    coh.mult(5.0);
    // Add the force vectors to acceleration
    this.applyForce(sep);
    this.applyForce(ali);
    this.applyForce(coh);
}

// Method to update location
Boid.prototype.update = function () {
    // Update velocity
    this.velocity.add(this.acceleration);
    // Limit speed
    this.velocity.limit(this.maxspeed);
    this.position.add(this.velocity);
    // Reset accelertion to 0 each cycle
    this.acceleration.mult(0);
}

// A method that calculates and applies a steering force towards a target
// STEER = DESIRED MINUS VELOCITY
Boid.prototype.seek = function (target) {
    var desired = p5.Vector.sub(target, this.position); // A vector pointing from the location to the target
    // Normalize desired and scale to maximum speed
    desired.normalize();
    desired.mult(this.maxspeed);
    // Steering = Desired minus Velocity
    var steer = p5.Vector.sub(desired, this.velocity);
    steer.limit(this.maxforce); // Limit to maximum steering force
    return steer;
}

// Draw boid as a person
Boid.prototype.render = function () {
    image(person, this.position.x, this.position.y, person.width/ 4, person.height/ 4);
}

// Wraparound
Boid.prototype.borders = function () {
    if (this.position.x < -this.r) this.position.x = width + this.r;
    if (this.position.y < -this.r) this.position.y = height + this.r;
    if (this.position.x > width + this.r) this.position.x = -this.r;
    if (this.position.y > height + this.r) this.position.y = -this.r;
}

// Separation
// Method checks for nearby boids and steers away
Boid.prototype.separate = function (boids) {
    var desiredseparation = 25.0;
    var steer = createVector(0, 0);
    var count = 0;
    // For every boid in the system, check if it's too close
    for (var i = 0; i < boids.length; i++) {
        var d = p5.Vector.dist(this.position, boids[i].position);
        // If the distance is greater than 0 and less than an arbitrary amount (0 when you are yourself)
        if ((d > 0) && (d < desiredseparation)) {
            // Calculate vector pointing away from neighbor
            var diff = p5.Vector.sub(this.position, boids[i].position);
            diff.normalize();
            diff.div(d); // Weight by distance
            steer.add(diff);
            count++; // Keep track of how many
        }
    }
    // Average -- divide by how many
    if (count > 0) {
        steer.div(count);
    }

    // As long as the vector is greater than 0
    if (steer.mag() > 0) {
        // Implement Reynolds: Steering = Desired - Velocity
        steer.normalize();
        steer.mult(this.maxspeed);
        steer.sub(this.velocity);
        steer.limit(this.maxforce);
    }
    return steer;
}

// Alignment
// For every nearby boid in the system, calculate the average velocity
Boid.prototype.align = function (boids) {
    var neighbordist = 50;
    var sum = createVector(0, 0);
    var count = 0;
    for (var i = 0; i < boids.length; i++) {
        var d = p5.Vector.dist(this.position, boids[i].position);
        if ((d > 0) && (d < neighbordist)) {
            sum.add(boids[i].velocity);
            count++;
        }
    }
    if (count > 0) {
        sum.div(count);
        sum.normalize();
        sum.mult(this.maxspeed);
        var steer = p5.Vector.sub(sum, this.velocity);
        steer.limit(this.maxforce);
        return steer;
    } else {
        return createVector(0, 0);
    }
}

// Cohesion
// For the average location (i.e. center) of all nearby boids, calculate steering vector towards that location
Boid.prototype.cohesion = function (boids) {
    var neighbordist = 50;
    var toWhere = createVector(0, 0); // Start with empty vector to accumulate all locations
    var count = 0;
    /*
    for (var i = 0; i < boids.length; i++) {
        var d = p5.Vector.dist(this.position, boids[i].position);
        if ((d > 0) && (d < neighbordist)) {
            sum.add(boids[i].position); // Add location
            count++;
        }
    }*/
    var min = 10000;
    for (var i = 0; i < moneys.length; i++) {
        var d = p5.Vector.dist(this.position, moneys[i].position);
        if ((d > 0) && (d < min)) {
            toWhere = moneys[i].position;
            min = d;
            count++;
        }    
    }
    if (count > 0) {
        //sum.div(count);
        return this.seek(toWhere); // Steer towards the location
    } else {
        return createVector(0, 0);
    }
}