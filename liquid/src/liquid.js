var bottle = null;
var canv_bottle1 = document.getElementById('display');
//var cbx = canv_bottle1.getContext("2d");

window.addEventListener('load', function () {
    bottle = new Bottle(document.getElementById('display'));

    function step() {
        bottle.step();
    }

    function render() {
        bottle.render();
        window.requestAnimationFrame(render);
    }
    window.requestAnimationFrame(render);
    setInterval(step, 1000 / Bottle.FPS);
});

canv_bottle1.addEventListener("mousedown",doMouseDown,false);
canv_bottle1.addEventListener("mousemove",doMouseMove,false);
canv_bottle1.addEventListener("mouseup",doMouseUp,false);


function doMouseDown(event)
{
    bottle.mouseDown(event.pageX,event.pageY);
}
function doMouseMove(event)
{
    bottle.mouseMove(event.pageX,event.pageY);
}

function doMouseUp(event)
{
    bottle.mouseUp(event.pageX,event.pageY);
}
