import math


def u_fun_p(A, B, C):
    return (B + math.sqrt(pow(A, 2) + pow(B, 2) - pow(C, 2))) / (C + A)


def u_fun_n(A, B, C):
    return (B - math.sqrt(pow(A, 2) + pow(B, 2) - pow(C, 2))) / (C + A)


a = 0.293
b = 0.352
c = 0.352
d = 0.293
e = 0.08
f = 0.127
x = -0.3
y = 0.3
pointA1 = 2 * x * a
pointA2 = 2 * (x - e) * d
pointB1 = 2 * y * a
pointB2 = 2 * y * d
pointC1 = pow(x, 2) + pow(y, 2) + pow(a, 2) - pow(b, 2)
pointC2 = pow(x - e, 2) + pow(y, 2) + pow(d, 2) - pow(c, 2)
u1_p = u_fun_p(pointA1, pointB1, pointC1)
u2_p = u_fun_p(pointA2, pointB2, pointC2)
u1_n = u_fun_n(pointA1, pointB1, pointC1)
u2_n = u_fun_n(pointA2, pointB2, pointC2)
theta1_p = math.degrees(2 * math.atan(u1_p))
theta2_p = math.degrees(2 * math.atan(u2_p))
theta1_n = math.degrees(2 * math.atan(u1_n))
theta2_n = math.degrees(2 * math.atan(u2_n))
print(theta1_p, theta2_p, theta1_n, theta2_n)
