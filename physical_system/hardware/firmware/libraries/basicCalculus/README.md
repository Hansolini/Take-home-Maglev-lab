# BasicCalculus
Simple library to _approximate 1-dimensional calculus operations_ in real time. It uses an _alpha-beta filter_ to __smoothen__ and numerically __differentiate__ a variable. The library can also __integrate__ the variable should one choose to. Note the library relies on polling to provide a continuously changing variable to refresh the calculus operations.

![image](http://hyperphysics.phy-astr.gsu.edu/hbase/Math/immath/derint.png)

# Other
For the sake of simplicity and to reduce memory use, the library uses micros() to update the timestep. This step is shared by all instances of alpha-beta filter.

# References:
- [Alpha-beta filter](https://en.wikipedia.org/wiki/Alpha_beta_filter)
- [Derivatives and Integrals](http://hyperphysics.phy-astr.gsu.edu/hbase/Math/derint.html)
- [Trapezoidal rule](https://en.wikipedia.org/wiki/Trapezoidal_rule)

