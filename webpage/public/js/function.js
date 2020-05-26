$(document).ready(function () {
  $("#gotoindex").on("click", function () {
    location.href = "../index.html";
  });
  $("#gotoorderer").on("click", function () {
    setTimeout(function () {
      console.log("sda");
      location.href = "html/orderer.html";
    }, 100);
  });
  $("#gotomaker").on("click", function () {
    setTimeout(function () {
      console.log("sda");
      location.href = "html/maker.html";
    }, 100);
  });
  $("#gotoseller").on("click", function () {
    setTimeout(function () {
      console.log("sda");
      location.href = "html/seller.html";
    }, 100);
  });
  $("#gotomap").on("click", function () {
    setTimeout(function () {
      console.log("sda");
      location.href = "html/map_justsee.html";
    }, 100);
  });

  $("#gotoorderer2").on("click", function () {
    setInterval(500);
    location.href = "orderer.html";
  });

  $("#gotoorder").on("click", function () {
    location.href = "order.html";
  });
  $("#gotoorderlist").on("click", function () {
    location.href = "orderlist.html";
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

  $("#goback").on("click", function () {
    history.go(-1);
    location.reload;
  });
  $("#gototest").on("click", function () {
    location.href = "html/test.html";
  });
});
//////////////////////////////////////////////////////
