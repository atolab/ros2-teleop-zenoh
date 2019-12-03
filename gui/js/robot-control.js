// $( "#arrow-up" ).mousedown(function() {
//   $.ajax({
//     type: "POST",
//     url: global_url.concat("/fwd"),
//     dataType: "json",
//     data: {}
//   }).done(function(data) {
//   });
// });

$( "#arrow-up" ).bind("mousedown touchstart", function() {
  $.ajax({
    type: "POST",
    url: global_url.concat("/fwd"),
    dataType: "json",
    data: {}
  }).done(function(data) {
  });
});

$( "#arrow-up" ).bind("mouseup touchend", function() {
  $.ajax({
    type: "POST",
    url: global_url.concat("/stop"),
    dataType: "json",
    data: {}
  }).done(function(data) {
  });
});

$( "#arrow-left" ).bind("mousedown touchstart", function() {
  $.ajax({
    type: "POST",
    url: global_url.concat("/sx"),
    dataType: "json",
    data: {}
  }).done(function(data) {
  });
});

$( "#arrow-left" ).bind("mouseup touchend", function() {
  $.ajax({
    type: "POST",
    url: global_url.concat("/stop"),
    dataType: "json",
    data: {}
  }).done(function(data) {
  });
});

$( "#arrow-right" ).bind("mousedown touchstart", function() {
  $.ajax({
    type: "POST",
    url: global_url.concat("/dx"),
    dataType: "json",
    data: {}
  }).done(function(data) {
  });
});

$( "#arrow-right" ).bind("mouseup touchend", function() {
  $.ajax({
    type: "POST",
    url: global_url.concat("/stop"),
    dataType: "json",
    data: {}
  }).done(function(data) {
  });
});

$( "#arrow-down" ).bind("mousedown touchstart", function() {
  $.ajax({
    type: "POST",
    url: global_url.concat("/bwd"),
    dataType: "json",
    data: {}
  }).done(function(data) {
  });
});

$( "#arrow-down" ).bind("mouseup touchend", function() {
  $.ajax({
    type: "POST",
    url: global_url.concat("/stop"),
    dataType: "json",
    data: {}
  }).done(function(data) {
  });
});
