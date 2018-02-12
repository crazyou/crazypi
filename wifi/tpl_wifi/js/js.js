$(function () {
    var public_ssid = '';
    T('.bb').on('tap', function (e) {
        e = e || window.event;
        var tag = e.target;
        if (tag && $(tag).hasClass('bb')) {
            if ($(tag).hasClass('else')) {
                $('.os,#forms #connect').hide();
                $('#forms,#forms #newConnect').fadeIn();
            }
            else if ($(tag).has('.os')){
                $('.os').fadeOut();
                $(tag).find('.os').fadeIn();
            }
        } else if (tag && $(tag).hasClass('cancel')) {
            $(tag).parent('.os').fadeOut();
        } else if (tag && $(tag).hasClass('close')) {
            //断开链接
            alert('断开连接');
        } else if (tag && $(tag).hasClass('connect')) {
            //连接
            public_ssid = $(this).find('.wifi-name').html();
            $('#forms #newConnect').hide();
            $('#forms,#forms #connect').fadeIn();
        }
    });
    //连接隐藏wifi 取消
    T('#btnCancel1').on('tap', function () {
        $('#forms #newsConnect').fadeOut();
        $('#forms').fadeOut();
    });
    //连接隐藏wifi 连接
    T('#btnConnect1').on('tap', function () {
        var ssid = $('#ssid').val(),
            pwd = $('#password1').val();
        if (ssid == '' || ssid.replace(/^\s*|\s*$/g, '').length < 1) {
            alert('请输入wifi名称'); return;
        }
        if (pwd == '' || pwd.replace(/^\s*|\s*$/g, '').length < 1) {
            alert('请输入密码'); return;
        }
        $('#password1,#ssid').blur();
        $('#forms #newsConnect').fadeOut();
        $('#forms').fadeOut();
        wifi_connect(ssid, pwd);
    });
    //连接wifi 取消
    T('#btnCancel').on('tap', function () {
        $('#forms #connect').fadeOut();
        $('#forms').fadeOut();
    });
    //连接wifi 连接
    T('#btnConnect').on('tap', function () {
        var pwd = $('#password').val();
        if (pwd == '' || pwd.replace(/^\s*|\s*$/g, '').length < 1) {
            alert('请输入密码'); return;
        }
        $('#password').blur();
        $('#forms #connect').fadeOut();
        $('#forms').fadeOut();
        wifi_connect(public_ssid, pwd);
    });
    function wifi_connect(s_ssid,s_pwd) {
        $('#connecting,#connecting .ing').fadeIn();
        $('#password,#password1,#ssid').val('');
        $.ajax({
            url: '/tryConnect',
            type: 'post',
            data: {
                ssid: s_ssid, password: s_pwd
            },
            dataType:'text',
            success: function (data) {
                $('#connecting .ing').hide();
                if (data && data * 1 == 1) {
                    $('#connecting .success').fadeIn(function () {
                        setTimeout(function () {
                            $('#connecting,#connecting .form').fadeIn();
                        }, 1000);
                    });
                } else {
                    $('#connecting .error').fadeIn(function () {
                        setTimeout(function () {
                            $('#connecting,#connecting .error').fadeOut();
                        }, 1000);
                    });
                }
            },
            error: function () {
                $('#connecting .ing').hide();
                $('#connecting .error').fadeIn(function () {
                    setTimeout(function () {
                        $('#connecting,#connecting .error').fadeOut();
                    }, 1000);
                });
            }
        });
    }
    T('#btnBind').on('tap', function () {
        var cname = $('#cname').val(),
            cpwd = $('#cpassword').val();
        if (cname == '' || cname.replace(/^\s*|\s*$/g, '').length < 1) {
            alert('请输入用户名'); return;
        }
        if (cpwd == '' || cpwd.replace(/^\s*|\s*$/g, '').length < 1) {
            alert('请输入密码'); return;
        }
        $('#cname,#cpassword').blur();
        $.ajax({
            url: '',
            type: 'get',
            dataType: 'jsonp',
            data: {
                username: cname, userpassword: cpwd
            }, success: function (data) {
                alert(data);
            },
            error: function () {
                alert('提示：登录失败');
            }
        });
    });

    var lp = {
        ePos: function (e) {
            var _isTouch = ('ontouchstart' in window);
            e = e || window.event;
            if (_isTouch) return e.originalEvent.touches[0].clientY;
            return  e.clientY || e.pageY;
        }
    };
    $('#list').on('touchstart', function (e) {
        lp.sy = lp.ePos(e);
        lp.ct = $('#list').offset().top;
    });
    $('#list').on('touchmove', function (e) {
        var cy = lp.ePos(e);
        if (Math.abs(cy - lp.sy) < 10) return;
        var dy = lp.ct + cy - lp.sy;
        dy = dy > 0 ? 0 : dy;
        var sh=-1*($('#list').height()-window.innerHeight);
        dy = dy < sh ? sh : dy;
        $('#list').offset({ top: dy });
    });
    $('#list').on('touchend', function (e) {});
});