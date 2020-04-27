
$(document).ready(function () { 
    $("#gotoindex").on("click", function () {
        location.href="index.html" 
    });
    $("#gotoorderer").on("click", function () {
        location.href="orderer.html" 
    });
    $("#gotoseller").on("click", function () {
        location.href="seller.html" 
    });
    $("#gotomain").on("click", function () { 
        location.href="index.html" 
    }); 
    $("#gotoorder").on("click", function () {
        location.href="order.html" 
    });
    $("#gotoorderlist").on("click", function () {
        location.href="orderlist.html" 
    });
    $("#gotomap").on("click", function () {
        location.href="map.html" 
    });
    $("#gotopassword").on("click", function () {
        location.href="password.html" 
    });
    $("#gotomaker").on("click", function () {
        location.href="maker.html" 
    });
    $("#goback").on("click", function(){
        history.go(-1)
    });



    $("#gotest").on("click", function(){
        var express = require('express');
        var router = express.Router();
 
/* GET home page. */
router.get('/',function(req, res, next) {
  res.render('index', { title:'Express' });
});
 
router.get('test',function(req, res, next) {
      res.render('test');
});
 
module.exports = router;
    });
}); 

//////////////////////////////////////////////////////

