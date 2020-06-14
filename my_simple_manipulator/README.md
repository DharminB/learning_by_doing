# My simple manipulator

## DH parameters

|   i   | alpha(i-1) |  a(i) |  d(i) | theta(i) |
| :---: | :--------: | :---: | :---: | :------: |
|   1   |   0        |   0   |   L1  |  theta1  |
|   2   | pi/2       |   0   |   0   |  theta2+pi/2  |
|   3   |   0        |   L2  |   0   |  theta3  |
|  ee   |   0        |   L3  |   0   |   0      |

```
x = ((L3*c3 + L2)*(c1*c2)) + (L3*s3*(-c1)*s2)
y = ((L3*c3 + L2)*(s1*c2)) + (L3*s3*(-s1)*s2)
z = ((L3*c3 + L2)*(s2)) + (L3*s3*c2) + L1
```

