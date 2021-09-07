# /usr/bin/env python
#-*- coding: utf-8 -*-

'''
드론의 구동알 알리는 문자 전송
'''
from twilio.rest import Client

account_sid = 'AC3a674bf50d4d0511d8600550e6e50739'   #twlilo 에서 받은 개인키
auth_token = '715e9cbd3d7bbd699606e9915362381c'      #twlilo 에서 받은 개인키
client = Client(account_sid, auth_token)


message = client.messages.create(
      body='왜 오류 뜨냐 시부럴!',      #메시지 내용
      from_='+17378885431',                           #텔로 공식 번호
      to = '+821077572419'                            #받을 문자 번호
          )

print(message.sid)
