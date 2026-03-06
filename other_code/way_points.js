var originX = 550;
var originY = 300;
var scaleFactor = 8;
var metersPerFoot= 0.3048;
var screenToWorld = function(x, y){
    return new PVector(
        (x - originX) / scaleFactor,
        (originY - y) / scaleFactor
    );
};

var worldToScreen = function(x, y){
    return new PVector(
        originX + x * scaleFactor,
        originY - y * scaleFactor
    );
};
var wayp = function(x,y,r){
    this.spot = new PVector(x,y);
    this.r = r;
};

wayp.prototype.draw = function(){
    var p = worldToScreen(this.spot.x, this.spot.y);
    noStroke();
    ellipse(p.x, p.y, this.r*2*scaleFactor, this.r*2*scaleFactor);
};

wayp.prototype.place = function(x,y){
    this.spot.set(x,y);
};
var path = function(spots){
    this.spots = spots;
};

path.prototype.draw = function(){
    fill(255);

    this.spots[0].draw();

    for(var i=1;i<this.spots.length;i++){

        var a = worldToScreen(
            this.spots[i-1].spot.x,
            this.spots[i-1].spot.y
        );

        var b = worldToScreen(
            this.spots[i].spot.x,
            this.spots[i].spot.y
        );

        strokeWeight(this.spots[i-1].r * scaleFactor);
        stroke(255);
        line(a.x,a.y,b.x,b.y);

        this.spots[i].draw();
    }
};
path.prototype.long=function(){
    var ans=0;
    for(var i=1;i<this.spots.length;i++){
        ans+=dist(this.spots[i-1].spot.x,this.spots[i-1].spot.y,this.spots[i].spot.x,this.spots[i].spot.y);
    }
    return ans;
};
var barrel=function(spot,col){
    this.spot=spot;
    this.col=col;
};
barrel.prototype.draw= function() {
    fill(this.col[0],this.col[1],this.col[2]);
    this.spot.draw();
};
var drawGrid = function(){
    stroke(80);
    strokeWeight(1);

    for(var x=-100;x<=100;x+=5){
        var a = worldToScreen(x,-100);
        var b = worldToScreen(x,100);
        line(a.x,a.y,b.x,b.y);
    }

    for(var y=-100;y<=100;y+=5){
        var a = worldToScreen(-100,y);
        var b = worldToScreen(100,y);
        line(a.x,a.y,b.x,b.y);
    }
};
var barrels=[
    new barrel(new wayp(0,-20,0.5),[255,255,0]),
    new barrel(new wayp(-30,-15,0.5),[255,255,0]),
    new barrel(new wayp(-30,-5,0.5),[0,0,255]),
    new barrel(new wayp(-60,-20,0.5),[255,255,0]),
    new barrel(new wayp(-60,20,0.5),[255,255,0]),
    new barrel(new wayp(0,20,0.5),[255,255,0]),
    new barrel(new wayp(-30,17,0.5),[255,0,0]),
    new barrel(new wayp(-30,22,0.5),[255,0,0])
];
var trail=new path([new wayp(-0,0,0.5)]);
draw= function() {
    background(155, 155, 155);
    drawGrid();
    trail.draw();
    for(var i=0; i<barrels.length;i++){
    barrels[i].draw();
    }
};
keyPressed=function(){
    if(key.toString()==="n"){
        var w = screenToWorld(mouseX,mouseY);
        trail.spots.push(new wayp(w.x,w.y,0.5));
    }else if(key.toString()===" "){
        for(var i=0;i<trail.spots.length;i++){
            println("\t{"+trail.spots[i].spot.y*metersPerFoot+","+trail.spots[i].spot.x/metersPerFoot+"},");
        }
    }
};
mouseDragged=function(){

    var w = screenToWorld(mouseX,mouseY);
    println(w.x+","+w.y);
    for(var i=0;i<trail.spots.length;i++){

        var d = dist(
            w.x, w.y,
            trail.spots[i].spot.x,
            trail.spots[i].spot.y
        );

        if(d < trail.spots[i].r*5){
            trail.spots[i].place(w.x,w.y);
        }
    }
};
