$(document).ready(function () {
  $("#gotoindex").on("click", function () {
    location.href = "../index.html";
  });
  $("#gotoorderer").on("click", function () {
    location.href = "html/orderer.html";
  });
  $("#gotoorderer2").on("click", function () {
    location.href = "orderer.html";
  });
  $("#gotoseller").on("click", function () {
    location.href = "html/seller.html";
  });
  $("#gotoorder").on("click", function () {
    location.href = "order.html";
  });
  $("#gotoorderlist").on("click", function () {
    location.href = "orderlist.html";
  });
  $("#gotomap").on("click", function () {
    location.href = "html/map_justsee.html";
  });
  $("#gotomap_robot").on("click", function () {
    location.href = "robotmap.html";
  });
  $("#gotopassword").on("click", function () {
    location.href = "password.html";
  });
  $("#gotolock").on("click", function () {
    location.href = "lock.html";
  });
  $("#gotomaker").on("click", function () {
    location.href = "html/maker.html";
  });
  $("#goback").on("click", function () {
    history.go(-1);
    location.reload;
  });
  $("#gototest").on("click", function () {
    location.href = "html/test.html";
  });
});
//////////////////////////////////////////////////////
