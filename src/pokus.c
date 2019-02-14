#include <stdio.h>


        double getCL(double alpha)
        {
            double alpha_degree=alpha*360.0/(2.0*3.141592);
            double CL=0.0;
            if(alpha_degree > 10  && alpha_degree <= 35)
                CL=(alpha_degree-10)*0.004+0.7;
            if(alpha_degree > 35 && alpha_degree < 90)
                CL=(alpha_degree-35)*-0.014+0.8;
            return CL;
        };

        double getCD(double alpha)
        {
            double alpha_degree=alpha*360.0/(2.0*3.141592);
            double CD=0.0;
            if(alpha_degree > -180  && alpha_degree <= -90)
                CD=(alpha_degree+180)/90*(-1.2);
            if(alpha_degree > -90  && alpha_degree <= 90)
                CD=alpha_degree/90*1.2;
            if(alpha_degree > 90 && alpha_degree <= 180)
                CD=(alpha_degree-90)/90*-1.2+1.2;
            return CD;
        };

void main(void)
{
    printf("degree;rad;CL;CD\n");
    for(int i=-180;i<180;i++)
    {
        double rad=(double)i*2.0*3.141592654/360.0;
        printf("%d;%f;%f;%f\n",i,rad,getCL(rad),getCD(rad));
    }

}
