# UE4SplineHelper
UE4 Spline Helper Prototype
This is A Contionus Smooth Curve Construction(Combining Multiple Bezier or Hermite Curves using Catmull-Rom):
In my Blueprint Libary C++, you can easily switch bezier with Hermite spline with auto construction of those tangents.

Your input should always be set of TArray<FVector> Control points, tangent should be auto generated for smooth curve using Catmull-Rom.

![alt text](https://github.com/tigershan1130/UE4SplineHelper/blob/master/ConstructBezierWithCatmullRomMod.jpg)

The project only shows how to generate and display tangent handle points and control points, if you want to actually modify those points in runtime or editor, you should implement your own solution.
