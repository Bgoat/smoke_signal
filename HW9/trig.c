#include <stdio.h>

int main(void)
{
	float degrees, radians;
	printf("Enter angles in degrees :");
	scanf("%f",&degrees);
	radians = 0.0174532925*degrees;
	printf("%.2f degrees = %f radians", degrees,radians);
	return(0);
}
