#define _CRT_SECURE_NO_WARNINGS
#include <stdio.h>

int main()
{
    int heading = 0;
    int target = 0;
    int error = 0;
    int Corrected_Error = 0;

    printf("Input Heading Angle = ");
    scanf("%d", &heading);

    printf("Input Target Angle = ");
    scanf("%d", &target);

    error = target - heading;

    printf("\n\n");
    printf("Error  : %d  =  %d  -  %d", error, target, heading);


    printf("\n\n");
    if (error < 180 && error >0)
    {
        Corrected_Error = target - heading;
        printf("Corrected Error  : %d  =  %d  -  %d", Corrected_Error, target, heading);
    }
    else if (error > 180)
    {
        Corrected_Error = error - 360;
        printf("Corrected Error  : %d  =  %d  -  %d", Corrected_Error, target, heading);
    }
    else if (error < 0 && error > -180)
    {
        Corrected_Error = error;
        printf("Corrected Error  : %d  =  %d  -  %d", Corrected_Error, target, heading);
    }
    else if (error < -180 && error > -360)
    {
        Corrected_Error = 360 + error;
        printf("Corrected Error  : %d  =  %d  -  %d", Corrected_Error, target, heading);
    }
    else if (error == 0 || error == 360 || error == -360)
    {
        Corrected_Error = 0;
        printf("Corrected Error  : %d  =  %d  -  %d", Corrected_Error, target, heading);
    }
    else if (error == 180 || error == -180)
    {
        Corrected_Error = 180;
        printf("Corrected Error  : %d  =  %d  -  %d", Corrected_Error, target, heading);
    }
    printf("\n\n");

}