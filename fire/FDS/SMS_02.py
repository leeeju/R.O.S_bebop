#!/usr/bin/env python/php
#-*- coding: utf-8 -*-
# https://console.coolsms.co.kr/
<?php
require_once("coolsms.php");
$sms = new coolsms();
$sms->setRealMode();
$sms->appversion("TEST/1.0");
$sms->charset("euckr");
$sms->setuser("02stu4@gmail.com", "wngusdl1!@");

if (!$sms->addsms("$argv[1]", "보내는 사람", "$argv[2]")) {
    echo $sms->lasterror();
}

if (!$sms->connect()) {
    exit(1);
}
$nsent = $sms->send();
if ($sms->errordetected()) {
}

$sms->disconnect();
$sms->emptyall();
?>

#*스크립트 종료*#
