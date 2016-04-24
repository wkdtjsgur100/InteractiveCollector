var boids = [];
var moneys = [];
var tosses = [];
var moneyPic;
var person;
var spark;
var tossPic;
var happyPerson;
var sparkTime;

function setup() {
    createCanvas(window.innerWidth, window.innerHeight);
    person = loadImage("../img/moneyeater/person.png");
    happyPerson = loadImage("../img/moneyeater/happyperson.png");
    moneyPic = loadImage("../img/moneyeater/50000s.png");
    spark = loadImage("../img/moneyeater/spark.png");
    tossPic = loadImage("../img/moneyeater/toss.png");
    sparkTime = 0;
    
    // Add an initial set of boids into the system
    for (var i = 0; i < 5; i++) {
        boids[i] = new Boid(random(width), random(height));
    }
}

function draw() {
    // Repaint gray on top each frame
    background(51);
    
    // Run all the boids
    for (var i = 0; i < boids.length; i++) {
        boids[i].run(boids);
    } 

     // Run all Tosses
    for (var i = 0; i < tosses.length; i++) {
        tosses[i].run(boids);
    }
    
    // Delete tosses that got kicked out
    var newTosses = [];
    for (var i = 0; i < tosses.length; i++) {
        if (tosses[i] != null) {
            newTosses.push(tosses[i]);
        }
    }
    tosses = newTosses;
    
    // Run all the moneys
    for (var i = 0; i < moneys.length; i++) {
        moneys[i].run(boids);
    }
    
    //Delete any moneys that got eaten
    var newMoneys = [];
    for (var i = 0; i < moneys.length; i++) {
        if (moneys[i] != null) {
            newMoneys.push(moneys[i]);
        }
    }
    moneys = newMoneys;
    
    // Change sparkling
    sparkTime++;
}

// Add new money/toss at the clicked location
function mousePressed() {
    if (Math.random() >= 0.2) {
        moneys[moneys.length] = new Money(mouseX, mouseY);    
    } else {
        tosses[tosses.length] = new Toss(mouseX, mouseY);
    }
}


// Money class
//////////////////////////////////////////////
// Creates the money class with current position
function Money(x, y) {
    this.position = createVector(x, y);
}

// Run for each frame
Money.prototype.run = function (boids) {
    // Make the index null if it is eaten
    if (this.checkEaten(boids)) {
        var index = moneys.indexOf(this);
        moneys[index] = null;
    } else {
        // If not eaten render
        this.render();
    }
}

// Check if the money should be deleted
Money.prototype.checkEaten = function (boids) {
    var eatingRadius = 75;
    var eaten = false;
    // Check if there's a person nearby
    for (var i = 0; i < boids.length; i++) {
        var d = p5.Vector.dist(this.position, boids[i].position);
        if (d < eatingRadius) {
            eaten = true;
        }
    }
    return eaten;
}

// Render draw money with spark
Money.prototype.render = function () {
    image(moneyPic, this.position.x, this.position.y, moneyPic.width/2, moneyPic.height/2);
    if ((Math.floor(sparkTime/10) % 2) == 0) {
        image(spark, this.position.x + 60, this.position.y - 30, spark.width/8, spark.height/8);
    }
}

// Toss Class
//////////////////////////////////////////////
// Create toss coins at the specified location
function Toss(x,y) {
    this.position = createVector(x,y);
    this.kicked = false;
    this.velocity = createVector(0,0);
}

// Run for each frame
Toss.prototype.run = function (boids) {
    // Check if the toss coin is kicked by boids
    if (this.checkKicked(boids)) {
        this.kicked = true;
    }
    // If the toss is kick, move the coin
    if (this.kicked) {
        this.position.add(this.velocity);
    }
    
    // Draw the toss image
    this.render();
    
    // If the coin gets out of the screen, delete the coin
    if (this.position.x < -this.r || this.position.x > width + this.r || this.position.y < -this.r || this.position.y > height + this.r) {
        var index = tosses.indexOf(this);
        tosses[index] = null;
    }
}

// Check if the toss should be kicked
Toss.prototype.checkKicked = function (boids) {
    var kickingRadius = 75;
    var kicked = false;
    for (var i = 0; i < boids.length; i++) {
        var d = p5.Vector.dist(this.position, boids[i].position);
        if (d < kickingRadius) {
            kicked = true;
            this.velocity = p5.Vector.sub(this.position, boids[i].position);
        }
    }
    return kicked;
}

// Render drawing toss coins for each frame
Toss.prototype.render = function () {
    image(tossPic, this.position.x, this.position.y, tossPic.width/4, tossPic.height/4);
}

// Boid class
////////////////////////////////////////////////
// Methods for Separation, Cohesion, Alignment added
function Boid(x, y) {
    this.acceleration = createVector(0, 0);
    this.velocity = p5.Vector.random2D();
    this.position = createVector(x, y);
    this.r = 3.0;
    this.maxspeed = 3; // Maximum speed
    this.maxforce = 0.05; // Maximum steering force
    this.angle = 0;
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
    //var ali = this.align(boids); // Alignment
    var coh = this.cohesion(boids); // Cohesion
    // Arbitrarily weight these forces
    sep.mult(3.0);
    //ali.mult(1.0);
    coh.mult(5.0);
    // Add the force vectors to acceleration
    this.applyForce(sep);
    //this.applyForce(ali);
    //this.velocity.limit(this.maxspeed);
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
    var left = createVector(-1, 0);
    var angle = p5.Vector.angleBetween(left, this.velocity);
    angle = (Math.floor(angle * 20 / PI))* PI / 20;
    push();
    angleMode(RADIANS);
    translate(this.position.x, this.position.y);
    this.angle = (0.9 * this.angle + angle * 0.1);
    if (this.velocity.x > 0) {
        scale(-1.0,1.0);
        if (this.velocity.y > 0) {
            rotate(this.angle - PI);    
        } else {
            rotate(PI - this.angle);
        }
    } else {
        if (this.velocity.y >= 0) {
            rotate(-this.angle);    
        } else {
            rotate(this.angle);    
        }
    }
 
    if(moneys.length > 0) {
        image(happyPerson, 0, 0, happyPerson.width/4, happyPerson.height/4);
  
    } else {
        image(person,0, 0, person.width/4, person.height/4);
    }
    pop();
}

// Bounce back if hit the wall
Boid.prototype.borders = function () {
    if (this.position.x < this.r) {
        this.velocity.x =  3;
    } 
    if (this.position.x > width - 30*this.r)
        this.velocity.x = -3;
    if (this.position.y < this.r)
        this.velocity.y = 3;
    if (this.position.y > height - 30*this.r)
        this.velocity.y = -3;
}

// Separation
// Method checks for nearby boids and steers away
Boid.prototype.separate = function (boids) {
    var desiredseparation = 100.0;
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
    var toWhere = createVector(0, 0); // Start with empty vector to accumulate all locations
    var count = 0;
    var min = 10000;
    // Find the closest money
    for (var i = 0; i < moneys.length; i++) {
        var d = p5.Vector.dist(this.position, moneys[i].position);
        if ((d > 0) && (d < min)) {
            toWhere = moneys[i].position;
            min = d;
            count++;
        }    
    }
    if (count > 0) {
        return this.seek(toWhere); // Steer towards the location
    } else {
        return createVector(0,0);
        //return createVector(random(-0.003,0.003), random(-0.003,0.003)); // No effect if there's no money
    }
}