# TODO:

* [x] add circle tool;
* [x] add third trajectory;
* [x] add slicing trajectory -> from line and circle, create set of points;
* [x] solve "trajectory not going where it should" problem;
* [x] wryyyyyyyyyyyyyyyyyyyy;
* [ ] refactor and comment;
* [x] add to point class add and sub method to implement line and circle trajectories;
* [x] solve why sometimes some trajectories are ignored;
* [x] TODOS in main.js and manipulator.js;
* [-> not a bug] solve bug that adds random circumferences sometimes :/


* [x] reimplement the spline3 method to use the final time formula from the cycloidal trajectory;
* [x] solve bug with spline3 that ignores last trajectory segment
* [ ] solve bug with circumferences when slicing it;


## 16/05/23
* [ ] professor suggested to implement linear interpolation between each q value obtained from ik-ing the sliced trajectory, with a set timing law whose duration depends on the maximum speed and acceleration of the motors (remember also the reductions) // leave the pen up movement as it is;
  * [ ] Move slicing to python // aka reimplement everything from scratch;
  * [ ] Implement timing law s(t) with different shapes (for example, linear or trapezoidal);
  * [ ] Implement trajectory stitching with linear stitches -> velocity and acceleration will be both constant within each "stitch" (stitch = line that connects q[i] to q[i+1]);
* [->] refactor and comment;
* [->] solve bug with circumferences when slicing it;