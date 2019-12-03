$( "#arrow-up" ).on("mousedown", "touchend", function() {
  $.ajax({
    type: "POST",
    url: global_url.concat("/fwd"),
    dataType: "json",
    data: {}
  }).done(function(data) {
  });
});

$( "#arrow-up" ).on("mouseup", "touchstart", function() {
  $.ajax({
    type: "POST",
    url: global_url.concat("/stop"),
    dataType: "json",
    data: {}
  }).done(function(data) {
  });
});

$( "#arrow-left" ).on("mousedown", "touchend", function() {
  $.ajax({
    type: "POST",
    url: global_url.concat("/sx"),
    dataType: "json",
    data: {}
  }).done(function(data) {
  });
});

$( "#arrow-left" ).on("mouseup", "touchstart", function() {
  $.ajax({
    type: "POST",
    url: global_url.concat("/stop"),
    dataType: "json",
    data: {}
  }).done(function(data) {
  });
});

$( "#arrow-right" ).on("mousedown", "touchend", function() {
  $.ajax({
    type: "POST",
    url: global_url.concat("/dx"),
    dataType: "json",
    data: {}
  }).done(function(data) {
  });
});

$( "#arrow-right" ).on("mouseup", "touchstart", function() {
  $.ajax({
    type: "POST",
    url: global_url.concat("/stop"),
    dataType: "json",
    data: {}
  }).done(function(data) {
  });
});

$( "#arrow-down" ).on("mousedown", "touchend", function() {
  $.ajax({
    type: "POST",
    url: global_url.concat("/bwd"),
    dataType: "json",
    data: {}
  }).done(function(data) {
  });
});

$( "#arrow-down" ).on("mouseup", "touchstart", function() {
  $.ajax({
    type: "POST",
    url: global_url.concat("/stop"),
    dataType: "json",
    data: {}
  }).done(function(data) {
  });
});
