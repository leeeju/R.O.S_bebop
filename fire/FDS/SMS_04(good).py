# /usr/bin/env python
#-*- coding: utf-8 -*-

from twilio.rest import Client

account_sid = 'AC3a674bf50d4d0511d8600550e6e50739'   #twlilo 에서 받은 개인키
auth_token = '656d855e6a0fbb8de72552a730c73bc9'      #twlilo 에서 받은 개인키
client = Client(account_sid, auth_token)


message = client.messages.create(
      body='불이야 불!!신토불이야!!',      #메시지 내용
      from_='+17378885431',                           #텔로 공식 번호
      to = '+821077572419'                            #받을 문자 번호
          )

print(message.sid)
