$(document).ready(function () {
  $("#gotoindex").on("click", function () {
    location.href = "../index.html";
  });
  $("#gotoorderer").on("click", function () {
    setTimeout(function () {
      console.log("sda");
      location.href = "html/orderer.html";
    }, 150);
  });
  $("#gotocontrol").on("click", function () {
    setTimeout(function () {
      console.log("sda");
      location.href = "html/control.html";
    }, 150);
  });
  $("#gotomaker").on("click", function () {
    setTimeout(function () {
      console.log("sda");
      location.href = "html/maker.html";
    }, 150);
  });
  $("#gotoseller").on("click", function () {
    setTimeout(function () {
      console.log("sda");
      location.href = "html/seller.html";
    }, 150);
  });
  $("#gotomap").on("click", function () {
    setTimeout(function () {
      location.href = "html/map_justsee.html";
    }, 150);
  });

  $("#gotoorderer2").on("click", function () {
    setInterval(500);
    location.href = "orderer.html";
  });
  $("#gotoseller2").on("click", function () {
    setTimeout(function () {
      console.log("sda");
      location.href = "seller.html";
    }, 150);
  });
  $("#gotoorder").on("click", function () {
    setTimeout(function () {
      location.href = "order.html";
    }, 150);
  });
  $("#gotoorderlist").on("click", function () {
    location.href = "orderlist.html";
  });
  $("#gotorobotstatus").on("click", function () {
    setTimeout(function () {
      location.href = "robotstatus.html";
    }, 150);
  });
  $("#gotomap_robot").on("click", function () {
    location.href = "robotmap.html";
  });
  $("#gotomap_robot2").on("click", function () {
    location.href = "robotmaps.html";
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

function modalopen() {
  document.getElementById("modal").style.display = "block";
}

document.getElementById("modal_close_btn").onclick = function () {
  document.getElementById("modal").style.display = "none";
  location.href = "../index.html";
};
