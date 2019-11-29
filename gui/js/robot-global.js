// Functions
function rgb(color, opacity) {
  var alpha = opacity === undefined ? 1 : 1 - opacity;
  var tmp = color.slice();
  tmp.push(alpha);
  return "rgba(" + tmp.toString() + ")";
}

var color = {
	coral: rgb([228, 82, 82])
};

var global_refresh = 250;
var global_robot = {};

var global_url = "http://127.0.0.1:5000";