var view = new Swiper(".view", {
  spaceBetween: 15,
});
var nav = new Swiper(".nav", {
  spaceBetween: 30,
  slidesPerView: "auto",
  touchRatio: 1,
  centeredSlides: true,
  slideToClickedSlide: true,
  onSlideChangeEnd: function () {
    $(".nav,.bottom,.view").removeClass("active");
  },
});

$(".content").each(function () {
  var t = $(this),
    content = new Swiper(t, {
      scrollbar: t.find(".swiper-scrollbar"),
      direction: "vertical",
      slidesPerView: "auto",
      mousewheelControl: true,
      spaceBetween: 15,
      freeMode: true,
      grabCursor: true,
      onSliderMove: function (swiper) {
        if (swiper.activeIndex > 0) {
          $(".nav,.bottom,.view").addClass("active");
        } else {
          $(".nav,.bottom,.view").removeClass("active");
        }
      },
      onSlideChangeEnd: function (swiper) {
        if (swiper.activeIndex > 0) {
          $(".nav,.bottom,.view").addClass("active");
        } else {
          $(".nav,.bottom,.view").removeClass("active");
        }
      },
      onScroll: function (swiper) {
        if (swiper.activeIndex > 0) {
          $(".nav,.bottom,.view").addClass("active");
        } else {
          $(".nav,.bottom,.view").removeClass("active");
        }
      },
    });
});

view.params.control = nav;
nav.params.control = view;
