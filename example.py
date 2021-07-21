#!/usr/bin/python
# -*- coding: utf-8 -*-

import math   # 수학 모듈 필수 (삼각함수)


th = math.radians(10)   # 각도 10도를 라디안으로 변환하여 th 에 대입

print "θ =", th, "rad =", math.degrees(th), "°\n"  # 라디안과 각도 출력

print "sin =", math.sin(th)     # 사인 (Sine)
print "cos =", math.cos(th)     # 코사인(Cosine)
print "tan =", math.tan(th)     # 탄젠트(Tangent)

print

print "csc =", 1.0 / math.sin(th)   # 코시컨트(Cosecant, cosec)
print "sec =", 1.0 / math.cos(th)   # 시컨트(Secant)
print "cot =", 1.0 / math.tan(th)   # 코탄젠트(Cotangent)

'''
θ = 0.174532925199 rad = 10.0 °

sin = 0.173648177667
cos = 0.984807753012
tan = 0.176326980708

csc = 5.75877048314
sec = 1.01542661189
cot = 5.67128181962
'''
