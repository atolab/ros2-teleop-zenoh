$( "#arrow-up" ).mousedown(function() {
  $.ajax({
    type: "POST",
    url: global_url.concat("/fwd"),
    dataType: "json",
    data: {}
  }).done(function(data) {
  });
});

$( "#arrow-up" ).mouseup(function() {
  $.ajax({
    type: "POST",
    url: global_url.concat("/stop"),
    dataType: "json",
    data: {}
  }).done(function(data) {
  });
});

$( "#arrow-left" ).mousedown(function() {
  $.ajax({
    type: "POST",
    url: global_url.concat("/sx"),
    dataType: "json",
    data: {}
  }).done(function(data) {
  });
});

$( "#arrow-left" ).mouseup(function() {
  $.ajax({
    type: "POST",
    url: global_url.concat("/stop"),
    dataType: "json",
    data: {}
  }).done(function(data) {
  });
});

$( "#arrow-right" ).mousedown(function() {
  $.ajax({
    type: "POST",
    url: global_url.concat("/dx"),
    dataType: "json",
    data: {}
  }).done(function(data) {
  });
});

$( "#arrow-right" ).mouseup(function() {
  $.ajax({
    type: "POST",
    url: global_url.concat("/stop"),
    dataType: "json",
    data: {}
  }).done(function(data) {
  });
});

$( "#arrow-down" ).mousedown(function() {
  $.ajax({
    type: "POST",
    url: global_url.concat("/bwd"),
    dataType: "json",
    data: {}
  }).done(function(data) {
  });
});

$( "#arrow-down" ).mouseup(function() {
  $.ajax({
    type: "POST",
    url: global_url.concat("/stop"),
    dataType: "json",
    data: {}
  }).done(function(data) {
  });
});
