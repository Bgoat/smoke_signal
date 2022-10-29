// Bee Goethals
// Oct 29, 2022

#include <stdlib.h>
#include <stdio.h>
#include <math.h>

const int nMaxTerms=3;
double factorials[nMaxTerms];

double factorial(int n)
{
    if (n==1)
        return 1;
    else
        return (double)n * factorial(n - 1.0);
}
float deg2Rad(float degrees)
{
    float radians = 0.0174532925*degrees;
    return radians;
}
void precalcFactorials()
{
    for (int i=1; i<nMaxTerms+1; i++)
    {
        factorials[i-1] = factorial(i);
    }
}

/*
    sin(x) = x - (x^3)/3! + (x^5)/5! - (x^7)/7! .......
*/
double taylorSine(double rads)
{
    double result = rads;

    for (int curTerm=1; curTerm<=(nMaxTerms/2)-1; curTerm++)
    {
        double curTermValue = pow(rads, (curTerm*2)+1);
        curTermValue /= factorials[ curTerm*2 ];
        if (curTerm & 0x01)
            result -= curTermValue;
        else
            result += curTermValue;
    }
    return result;
}

/*
    cos(x) = 1 - (x^2)/2! + (x^4)/4! - (x^6)/6! .......
*/
double taylorCos(double rads)
{
    double result = 1.0;
    for (int curTerm=1; curTerm<=(nMaxTerms/2)-1; curTerm++)
    {
        double curTermValue = pow(rads, (curTerm*2) );
        curTermValue /= factorials[ (curTerm*2) - 1 ];
        if (curTerm & 0x01)
            result -= curTermValue;
        else
            result += curTermValue;
    }
    return result;
}

float SineApproximation (float theta, float resolution)
{
    // calculate the stepDelta for our angle.
    // resolution is the number of samples we calculate from 0 to 2pi radians
    const float TwoPi = 6.28318530718f;
    const float stepDelta = (TwoPi / resolution);
 
    // initialize our starting values
    float angle = 0.0;
    float vcos = 1.0;
    float vsin = 0.0;
 
    // while we are less than our desired angle
    while(angle < theta) {
 
        // calculate our step size on the y axis for our step size on the x axis.
        float vcosscaled = vcos * stepDelta;
        float vsinscaled = vsin * stepDelta;
 
        // take a step on the x axis
        angle += stepDelta;
 
        // take a step on the y axis
        vsin += vcosscaled;
        vcos -= vsinscaled;
    }
    return vsin;
}

double clamp(double d, double min, double max) {
  const double t = d < min ? min : d;
  return t > max ? max : t;
}

float mySmoothstep(float edge0, float edge1, float x) {
  
    // Scale the value of x respect to edge0 edge1, and clamp in the interval [0.0, 1.0]  
  	x = clamp((x - edge0) / (edge1 - edge0), 0.0, 1.0); 
  	
    // Evaluate a polinomial
  	return x * x * (3. - 2. * x);
}

float SmoothstepSin(const float _x)
{
    // make a triangle wave that has y values from 0-1, where y is 0 at x=0
	float x = fabs(factorial((_x - deg2Rad(90.0)) / deg2Rad(360.0))*2.0-1.0);
       
    // smoothstep the triangle wave and then scale it to the -1 to 1 range
    return mySmoothstep(0.0,1.0,x) * 2.0 - 1.0;
    
	
    // Note that you could use this instead of smoothstep above.  Same thing.
    //float x2 = x*x; 
	//return ((3.0 * x2) - (2.0 * x2 * x)) * 2.0 - 1.0;
    
    // or this:
    //return x * x * (3.0 - 2.0 * x) * 2.0 - 1.0;
}

int main()
{
    precalcFactorials();
    printf("Math sin(0.5) = %f\n", sin(0.5));
    printf("taylorSin(0.5) = %f\n", taylorSine(0.5));
    printf("Slope Iteration Method(0.5,1020) = %f\n", SineApproximation(0.5,1020));
    printf("Smoothstep Method(0.5) = %f\n", SmoothstepSin(0.5));
  //  printf("Math cos(0.5) = %f\n", cos(0.5));
  //  printf("taylorCos(0.5) = %f\n", taylorCos(0.5));

    return 0;
}