Python 2.7.14 (v2.7.14:84471935ed, Sep 16 2017, 20:19:30) [MSC v.1500 32 bit (Intel)] on win32
Type "copyright", "credits" or "license()" for more information.
>>> import numpy as np
>>> y = np.array([1068, 988, 1002, 996])
>>> y = y.reshape((len(y), 1))
>>> y.shape
(4, 1)
>>> R = np.diag([400, 400, 4, 4])
>>> R.shape
(4, 4)
>>> x_hat = np.matmul
>>> H = np.ones((len(y),1))
>>> H.shape
(4, 1)
>>> x_hat = np.matmul(np.linalg.inv(np.matmul(H.T, np.matmul(np))))

Traceback (most recent call last):
  File "<pyshell#15>", line 1, in <module>
    x_hat = np.matmul(np.linalg.inv(np.matmul(H.T, np.matmul(np))))
TypeError: Required argument 'b' (pos 2) not found
>>> b = np.matmul(H.T, np.linalg.inv(R))
>>> x_hat = np.matmul( np.matmul(np.linalg.inv(np.matmul(b, H)), b), y)
>>> x_hat
array([[999.28712871]])
>>> 
