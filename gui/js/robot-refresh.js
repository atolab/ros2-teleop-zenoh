setInterval(function() {
  $.ajax({
    type: "GET",
		url: global_url.concat("/sensors"),
    dataType: "json",
	}).done(function(data) {
    $("#proximity").text(data.distance);
    $("#speed-linear").text(data.target_linear_velocity);
    $("#speed-angular").text(data.target_angular_velocity);
    $("#temperature").text(Math.floor(data.temperature));
    $("#humidity").text(Math.floor(data.humidity));
    $("#air-quality").text(data.air_quality);
	}).fail(function(data) {
    $("#proximity").text("NA");
    $("#speed-linear").text("NA");
    $("#speed-angular").text("NA");
    $("#temperature").text("NA");
    $("#humidity").text("NA");
    $("#air-quality").text("NA");
  });
}, global_refresh);
