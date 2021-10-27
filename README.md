# Fit Curve for C#

C# implementation of Philip J. Schneider's "Algorithm for Automatically Fitting Digitized Curves" from the book "Graphics Gems".   
Converted from javascript implementation from https://github.com/soswow/fit-curve

Fit one or more cubic Bezier curves to a polyline. Works with 3D curves.

This is a C# implementation of Philip J. Schneider's C code. The original C code is available on
http://graphicsgems.org/ as well as in https://github.com/erich666/GraphicsGems

## install

download and install *FitCurve.unitypackage*

## Usage
```
var points = new List<Vecotr3>() { new Vector3(0,0,0), new Vector3(0,10,10), new Vector3(10,-10,0), new Vector3(5,5,-4) };
var maxErrpr = 5; // The smaller the number - the much closer spline should be
var bezierPoints = FitCurveUtil.FitCurve(points, maxError, false);
```
