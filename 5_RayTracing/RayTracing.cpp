
// sudo apt-get install libeigen3-dev
// sudo apt-get install libpng-dev
// includePATH
// -lpng

#include <iostream>
#include <sstream>
#include <math.h>
#include <fstream>
#include <cstring>
#include <vector>
#include <eigen3/Eigen/Eigen>
#include <eigen3/Eigen/Dense>
using namespace Eigen;
using namespace std;
typedef Matrix<float, 3, 1> Vector3f;
static int width = 3000;
static int height = 3000;
static int maxColVal = 255;
static float refractionRate = 1.5;
int data[30000000];
float dist, dist_1, dist_2, dist_3;

struct ray
{
    Vector3f origin, target;

    static ray Ray(Vector3f o, Vector3f t) {
        ray R;
        R.origin = o;
        R.target = t;
        return R;
    }


    static float distanceFromPoint (ray R, Vector3f point) {
        Vector3f origin = R.origin;
        Vector3f target = R.target;

        Vector3f OP = point-origin;
        Vector3f a = (OP.cross(target)/target.norm());
        float d = OP.dot(target);
        if(d<0) return 100000.;
        else return a.norm();
    }

    static Vector3f contactPoint (ray R, Vector3f point, float r) {
        Vector3f origin = R.origin;
        Vector3f target = R.target;
        Vector3f X = origin - point;
        float A = target.squaredNorm();
        float B = X.dot(target);
        float C = X.squaredNorm() - r*r;
        float t = (-B-sqrt(B*B-A*C))/A;
        Vector3f contact = origin + t*target;
        return contact;
    }

    static ray reflectedRay (ray R, Vector3f point, float r) {
        Vector3f origin = R.origin;
        Vector3f target = R.target;
        Vector3f contact = ray::contactPoint(R, point, r);
        Vector3f PC = contact - point;
        Vector3f OP = point - origin;
        // origin에서 point - contact 직선에 내린 수선 벡터 V를 구해 O+2V 를 구하고, origin=contact, target=O+2V인 Ray return.
        Vector3f H = point - (OP.dot(PC) / PC.squaredNorm() ) * PC;
        Vector3f newTarget = origin + 2*(H-origin) - contact;
        ray newRay;
        newRay.origin = contact;
        newRay.target = newTarget;
        return newRay;



    }

    static Vector3f toZplane (ray R)
    {
        Vector3f ans;
        
        Vector3f o = R.origin;
        Vector3f t = R.target;
        if(t[2]==0) 
        {
            ans[0] = 10000.;
            ans[1] = 10000.;
            ans[2] = 10000.;
            return ans;
        }
        float x = o[2] / t[2];
        if(o[2]<t[2])
        {
            ans[0] = 10000.;
            ans[1] = 10000.;
            ans[2] = 10000.;
            return ans;
        }
        ans[0] = o[0]-x*t[0];
        ans[1] = o[1]-x*t[1];
        ans[2] = 0;
        return ans;
    }

    static ray sphereRefraction (ray R, Vector3f point, float r)
    {
        Vector3f origin = R.origin;
        Vector3f target = R.target;
        Vector3f contact = ray::contactPoint(R, point, r);
        Vector3f PC = contact - point;
        Vector3f OP = point - origin;
        // origin에서 point - contact 직선에 내린 수선 벡터 V를 구해 O+2V 를 구하고, origin=contact, target=O+2V인 Ray return.
        Vector3f H = point - (OP.dot(PC) / PC.squaredNorm() ) * PC;
        Vector3f OH = H-origin;
        Vector3f HC = contact - H;
        float theta = atan(OH.norm() / HC.norm());
        float theta_n = asin(sin(theta)/refractionRate);
        Vector3f CD = r*cos(theta_n)/cos(theta-theta_n)/target.norm()*target;
        Vector3f CE_vec = -PC+PC.norm()*tan(theta_n)/OH.norm()*OH;
        Vector3f CE = 2*r*cos(theta_n)/CE_vec.norm()*CE_vec;
        Vector3f DE = CE - CD;
        Vector3f D = contact + CD;

        ray newRay;
        newRay.origin = D;
        newRay.target = DE;
        return newRay;
    }

    static float diffuseRate(ray R, Vector3f point, float r)
    {
        


        Vector3f c = ray::contactPoint(R, point, r);
        float z = c[2];
        if(z<r) return 0;
        else 
        {
            float theta = asin((z-r)/r);

            return (z-r)/r;
        }
    }

    static Vector3f toX_30 (ray R)
    {
        Vector3f ans;
        
        Vector3f o = R.origin;
        Vector3f t = R.target;
        if(t[0]==-30) 
        {
            ans[0] = 10000.;
            ans[1] = 10000.;
            ans[2] = 10000.;
            return ans;
        }
        float x = (-30-o[0]) / t[0];
        if(x<0.)
        {
            ans[0] = 10000.;
            ans[1] = 10000.;
            ans[2] = 10000.;
            return ans;
        }
        ans[0] = -30;
        ans[1] = o[1]+x*t[1];
        ans[2] = o[2]+x*t[2];
        return ans;
    }

    static Vector3f toY20 (ray R)
    {
        Vector3f ans;
        
        Vector3f o = R.origin;
        Vector3f t = R.target;
        if(t[1]==20) 
        {
            ans[0] = 10000.;
            ans[1] = 10000.;
            ans[2] = 10000.;
            return ans;
        }
        float x = (20-o[1]) / t[1];
        if(x<0.)
        {
            ans[0] = 10000.;
            ans[1] = 10000.;
            ans[2] = 10000.;
            return ans;
        }
        ans[0] = o[0]+x*t[0];
        ans[1] = 20;
        ans[2] = o[2]+x*t[2];
        return ans;
    }

    static Vector3f toZ10 (ray R)
    {
        Vector3f ans;
        
        Vector3f o = R.origin;
        Vector3f t = R.target;
        if(t[2]==10) 
        {
            ans[0] = 10000.;
            ans[1] = 10000.;
            ans[2] = 10000.;
            return ans;
        }
        float x = (10-o[2]) / t[2];
        if(x<0.)
        {
            ans[0] = 10000.;
            ans[1] = 10000.;
            ans[2] = 10000.;
            return ans;
        }
        ans[0] = o[0]+x*t[0];
        ans[1] = o[1]+x*t[1];
        ans[2] = 30;
        return ans;
    }




};

float reverse (float dist)
{
    float rev = 1/dist;
    return rev>1. ? 1 : rev;
}

int main() {

    std::string RGBptr;
    std::ofstream outFile;

    outFile.open("Image.ppm", std::ios::out | std::ios::binary);
    outFile << "P6"     << "\n"
        << width       << " "
        << height      << "\n"
        << maxColVal   << "\n"
       ;

    Vector3f A, B, C, D;
    A[0] = 715.96;
    A[1] = 2348.96;
    A[2] = 158.71;
    B[1] = 715.96;
    B[0] = 2348.96;
    B[2] = 158.71;
    C[0] = 1568.69;
    C[1] = -64.31;
    C[2] = -1870;
    D[1] = 1568.69;
    D[0] = -64.31;
    D[2] = -1870;
    Vector3f origin;
    origin[0] = -100.;
    origin[1] = -100.;
    origin[2] = 100.;
    Vector3f bulb_1;
    bulb_1[0] = 150.;
    bulb_1[1] = 100.;
    bulb_1[2] = 100.;
    Vector3f bulb_2;
    bulb_2[0] = 250;
    bulb_2[1] = 100.;
    bulb_2[2] = 150.;
    Vector3f bulb_3;
    bulb_3[0] = 250.;
    bulb_3[1] = 100.;
    bulb_3[2] = 200.;
    Vector3f yellow;
    yellow[0]=50.;
    yellow[1]=70.;
    yellow[2]=0.;
    Vector3f red;
    red[0]=70.;
    red[1]=50.;
    red[2]=0.;
    Vector3f white;
    white[0]=80.;
    white[1]=80.;
    white[2]=0.;
    Vector3f Ref1;
    Ref1[0]=60.;
    Ref1[1]=18.;
    Ref1[2]=0.;
    Vector3f Ref2;
    Ref2[0]=18.;
    Ref2[1]=60.;
    Ref2[2]=0.;
    Vector3f Ref3;
    Ref3[0]=33.;
    Ref3[1]=33.;
    Ref3[2]=0.;

    int cnt = 0;
    float radius = 12.;
    float bulb_r = 50.;

    int k=0;

    float Ambient[3];
    float Diffuse[3];
    float Specular[3];
    float tmp;
    float rev;
    float diffrate;
    Vector3f v;


    for (int i=0; i<width; i++)
    {
        for(int j=0; j<height; j++)
        {
            float a = float(i)/float(width);
            float b = float(j)/float(height);
            Vector3f targetPos = (1.-a)*(1.-b)*A + (1.-a)*(b)*B + (a)*(b)* C +(a)*(1.-b)* D;
            Vector3f target = targetPos - origin;
            ray Ray = ray::Ray(origin, target);
           

            Vector3f vx = ray::toX_30(Ray);
            Vector3f vy = ray::toY20(Ray);
            Vector3f vz = ray::toZ10(Ray);

            if(vx[1]>20 && vx[1]<30 && vx[2]>0 && vx[2]<10)
            {
                Ambient[0] = 14*2;
                Ambient[1] = 58*2;
                Ambient[2] = 58*2;
                Diffuse[0] = 0;
                Diffuse[1] = 0;
                Diffuse[2] = 0;
                Specular[0] = 0;
                Specular[1] = 0;
                Specular[2] = 0;
            }
            else if(vy[0]>-30 && vy[0]<-20 && vy[2]>0 && vy[2]<10)
            {
                Ambient[0] = 14*2;
                Ambient[1] = 58*2;
                Ambient[2] = 58*2;
                Diffuse[0] = 20;
                Diffuse[1] = 20;
                Diffuse[2] = 20;
                Specular[0] = 0;
                Specular[1] = 0;
                Specular[2] = 0;
            }
            else if(vz[0]>-30 && vz[0]<-20 && vz[1]>20 && vz[1]<30)
            {
                float dist = sqrt( (vz[0]+25.0)*(vz[0]+25.0)+(vz[1]-25.0)*(vz[1]-25.0) );
                if(dist<3.0)
                {
                    float dist = sqrt( (vz[0]+26.0)*(vz[0]+26.0)+(vz[1]-26.5)*(vz[1]-26.5) );
                    Diffuse[0] = 40-dist*10;
                    Diffuse[1] = 40-dist*10;
                    Diffuse[2] = 40-dist*10;
                }
                else
                {
                    Diffuse[0] = 40;
                    Diffuse[1] = 40;
                    Diffuse[2] = 40;
                }

                Ambient[0] = 14*2;
                Ambient[1] = 58*2;
                Ambient[2] = 58*2;
                
                Specular[0] = 0;
                Specular[1] = 0;
                Specular[2] = 0;
            }   
            else
            {
                 // refraction first

                dist = ray::distanceFromPoint(Ray, Ref3);
                if(dist<radius)
                {
                    Ray = ray::sphereRefraction(Ray, Ref3, radius);
                }

                dist = ray::distanceFromPoint(Ray, Ref1);
                if(dist<radius)
                {
                    Ray = ray::sphereRefraction(Ray, Ref1, radius);
                }

                dist = ray::distanceFromPoint(Ray, Ref2);
                if(dist<radius)
                {
                    Ray = ray::sphereRefraction(Ray, Ref2, radius);
                }
                dist = ray::distanceFromPoint(Ray, yellow);
            
            if(dist<radius)
            {
                // light from yellow ball
                Ambient[0] = 255.*0.7;
                Ambient[1] = 255.*0.7;
                Ambient[2] = 0;
                
                // reflected
                diffrate = ray::diffuseRate(Ray, yellow, radius);
                ray Reflected = ray::reflectedRay(Ray, yellow, radius);
                dist_1 = ray::distanceFromPoint(Reflected, bulb_1);
                dist_2 = ray::distanceFromPoint(Reflected, bulb_2);
                dist_3 = ray::distanceFromPoint(Reflected, bulb_3);
                dist = min({(dist_1, dist_2, dist_3)});
                rev = reverse(dist);
                

                Diffuse[0] = 255.*0.35*diffrate;
                Diffuse[1] = 255.*0.35*diffrate;
                Diffuse[2] = 50.*0.15*diffrate;
                
                // 광원으로 들어가는지 확인
                if(dist<bulb_r)
                {
                    Specular[0] = 255;
                    Specular[1] = 255;
                    Specular[2] = 255;
                }
                else
                {
                    dist = ray::distanceFromPoint(Reflected, red);
                    rev = reverse(dist);
                    if(dist<radius)
                    {
                        Reflected = ray::reflectedRay(Reflected, red, radius);
                        dist_2 = ray::distanceFromPoint(Reflected, yellow);
                        float rev2 = reverse(dist_2);
                        if(dist_2<radius)
                        {
                            // eye -> yellow -> red -> yellow
                            Specular[0] = 255*(0.1+0.05*rev2);
                            Specular[1] = 255*(0.1+0.05*rev2);
                            Specular[2] = 0;
                        }
                        else
                        {
                            // eye -> yellow -> red
                            Specular[0] = 255*(0.1+0.05*rev);
                            Specular[1] = 0;
                            Specular[2] = 0;
                        }
                        
                    }
                    else
                    {
                        v = ray::toZplane(Reflected);
                        if(v[0]>0 && v[0]<1000 && v[1]>0 && v[1]<600)
                        {
                            Specular[0] = 0;
                            Specular[1] = 0;
                            Specular[2] = 255*0.3;
                        }

                        else if((v[0]>-50 && v[0]<0 && v[1]>-50 && v[1]<650) || (v[0]>-50 && v[0]<1050 && v[1]>-50 && v[1]<0) || (v[0]>1000 && v[0]<1050 && v[1]>-50 && v[1]<650) || (v[0]>-50 && v[0]<1050 && v[1]>600 && v[1]<650))
                        {
                            int row = v[0];
                                int col = v[1];
                                if((row+col)%2==1)
                                {
                                    Specular[0] = 66*0.5;
                                    Specular[1] = 29*0.5;
                                    Specular[2] = 20*0.5;
                                }
                                else
                                {
                                    Specular[0] = 66*1;
                                    Specular[1] = 29*1;
                                    Specular[2] = 20*1;
                                }
                        }

                        else
                        {
                            Specular[0] = 0;
                            Specular[1] = 0;
                            Specular[2] = 0;
                        }

                        
                    }
                    
                }
            }
            else
            {
                
                dist = ray::distanceFromPoint(Ray, red);
                if(dist<radius)
                {
                    // light from red
                    Ambient[0] = 255.*0.7;
                    Ambient[1] = 0;
                    Ambient[2] = 0;

                    // reflected
                    diffrate = ray::diffuseRate(Ray, red, radius);
                    ray Reflected = ray::reflectedRay(Ray, red, radius);
                    dist_1 = ray::distanceFromPoint(Reflected, bulb_1);
                    dist_2 = ray::distanceFromPoint(Reflected, bulb_2);
                    dist_3 = ray::distanceFromPoint(Reflected, bulb_3);
                    dist = min({(dist_1, dist_2, dist_3)});
                    rev = reverse(dist);

                    

                    Diffuse[0] = 255.*0.35*diffrate;
                    Diffuse[1] = 50.*0.35*diffrate;
                    Diffuse[2] = 50.*0.35*diffrate;

                    if(dist<bulb_r)
                    {
                        Specular[0] = 255;
                        Specular[1] = 255;
                        Specular[2] = 255;
                    }
                    else
                    {
                        dist = ray::distanceFromPoint(Reflected, yellow);
                        rev = reverse(dist);
                        if(dist<radius)
                        {

                            Reflected = ray::reflectedRay(Reflected, yellow, radius);
                            dist_2 = ray::distanceFromPoint(Reflected, red);
                            float rev2 = reverse(dist_2);
                            if(dist_2<radius)
                            {
                                // eye -> red -> yellow -> red
                                Specular[0] = 255*(0.1+0.05*rev2);
                                Specular[1] = 0;
                                Specular[2] = 0;
                            }
                            else
                            {
                                // eye -> red -> yellow
                                Specular[0] = 255*(0.1+0.05*rev);
                                Specular[1] = 255*(0.1+0.05*rev);
                                Specular[2] = 0;
                            }
                            
                        }
                        else
                        {
                            v = ray::toZplane(Reflected);
                            if(v[0]>0 && v[0]<1000 && v[1]>0 && v[1]<600)
                            {
                                Specular[0] = 0;
                                Specular[1] = 0;
                                Specular[2] = 255*0.3;
                            }

                            else if((v[0]>-50 && v[0]<0 && v[1]>-50 && v[1]<650) || (v[0]>-50 && v[0]<1050 && v[1]>-50 && v[1]<0) || (v[0]>1000 && v[0]<1050 && v[1]>-50 && v[1]<650) || (v[0]>-50 && v[0]<1050 && v[1]>600 && v[1]<650))
                            {
                                int row = v[0];
                                int col = v[1];
                                if((row+col)%2==1)
                                {
                                    Specular[0] = 66*0.5;
                                    Specular[1] = 29*0.5;
                                    Specular[2] = 20*0.5;
                                }
                                else
                                {
                                    Specular[0] = 66*1;
                                    Specular[1] = 29*1;
                                    Specular[2] = 20*1;
                                }
                            }

                            else
                            {
                                Specular[0] = 0;
                                Specular[1] = 0;
                                Specular[2] = 0;
                            }
                        }
                    }
                    // data[k] = Ambient[0]+Diffuse[0]+Specular[0];
                    // data[k+1] = Ambient[1]+Diffuse[1]+Specular[1];
                    // data[k+2] = Ambient[2]+Diffuse[2]+Specular[2];
                    // k=k+3;
                    
                }
                else
                {
                    
                    dist = ray::distanceFromPoint(Ray, white);
                    rev = reverse(dist);
                    if(dist<radius)
                    {
                        // light from white
                        Ambient[0] = 255.*0.7;
                        Ambient[1] = 255.*0.7;
                        Ambient[2] = 255.*0.7;
                        
                        // reflected
                        diffrate = ray::diffuseRate(Ray, white, radius);
                        ray Reflected = ray::reflectedRay(Ray, white, radius);
                        dist_1 = ray::distanceFromPoint(Reflected, bulb_1);
                        dist_2 = ray::distanceFromPoint(Reflected, bulb_2);
                        dist_3 = ray::distanceFromPoint(Reflected, bulb_3);
                        dist = min({(dist_1, dist_2, dist_3)});
                        rev = reverse(dist);

                        

                        Diffuse[0] = 255.*0.3*diffrate;
                        Diffuse[1] = 255.*0.3*diffrate;
                        Diffuse[2] = 255.*0.3*diffrate;

                        if(dist<bulb_r)
                        {
                            Specular[0] = 255;
                            Specular[1] = 255;
                            Specular[1] = 255;
                        }
                        else
                        {
                            dist_1 = ray::distanceFromPoint(Reflected, yellow);
                            dist_2 = ray::distanceFromPoint(Reflected, red);
                            v = ray::toZplane(Reflected);
                            if(dist_1<radius)
                            {
                                rev = reverse(dist_1);

                                Reflected = ray::reflectedRay(Reflected, yellow, radius);
                                dist_2 = ray::distanceFromPoint(Reflected, white);
                                float rev2 = reverse(dist_2);
                                if(dist_2<radius)
                                {
                                    // eye -> white -> yellow -> white
                                    Specular[0] = 255*(0.1+0.05*rev2);
                                    Specular[1] = 255*(0.1+0.05*rev2);
                                    Specular[2] = 255*(0.1+0.05*rev2);
                                }
                                else
                                {
                                    // eye -> white -> yellow
                                    Specular[0] = 255*(0.1+0.05*rev);
                                    Specular[1] = 255*(0.1+0.05*rev);
                                    Specular[2] = 0;
                                }

                            }
                            else if(dist_2<radius)
                            {
                                rev = reverse(dist_2);

                                Reflected = ray::reflectedRay(Reflected, red, radius);
                                dist_2 = ray::distanceFromPoint(Reflected, white);
                                float rev2 = reverse(dist_2);
                                if(dist_2<radius)
                                {
                                    // eye -> white -> red -> white
                                    Specular[0] = 255*(0.1+0.05*rev2);
                                    Specular[1] = 255*(0.1+0.05*rev2);
                                    Specular[2] = 255*(0.1+0.05*rev2);
                                }
                                else
                                {
                                    // eye -> white -> red
                                    Specular[0] = 255*(0.1+0.05*rev);
                                    Specular[1] = 0;
                                    Specular[2] = 0;
                                }




                                
                            }
                            else if(v[0]>0 && v[0]<1000 && v[1]>0 && v[1]<600)
                            {
                                Specular[0] = 0;
                                Specular[1] = 0;
                                Specular[2] = 255*0.2;
                            }

                            else if((v[0]>-50 && v[0]<0 && v[1]>-50 && v[1]<650) || (v[0]>-50 && v[0]<1050 && v[1]>-50 && v[1]<0) || (v[0]>1000 && v[0]<1050 && v[1]>-50 && v[1]<650) || (v[0]>-50 && v[0]<1050 && v[1]>600 && v[1]<650))
                            {
                                int row = v[0];
                                int col = v[1];
                                if((row+col)%2==1)
                                {
                                    Specular[0] = 66*0.5;
                                    Specular[1] = 29*0.5;
                                    Specular[2] = 20*0.5;
                                }
                                else
                                {
                                    Specular[0] = 66*1;
                                    Specular[1] = 29*1;
                                    Specular[2] = 20*1;
                                }
                            }

                            else
                            {
                                Specular[0] = 0;
                                Specular[1] = 0;
                                Specular[2] = 0;
                            }
                        }
                    }
                    else
                    {
                        v = ray::toZplane(Ray);
                        if(v[0]>0 && v[0]<1000 && v[1]>0 && v[1]<600)
                        {
                            Diffuse[0] = 0;
                            Diffuse[1] = 0;
                            Diffuse[2] = 255*0.8;

                            Ambient[0] = 0;
                            Ambient[1] = 0;
                            Ambient[2] = 20;

                            Specular[0] = 0;
                            Specular[1] = 0;
                            Specular[2] = 0;
                        }

                        else if((v[0]>-50 && v[0]<0 && v[1]>-50 && v[1]<650) || (v[0]>-50 && v[0]<1050 && v[1]>-50 && v[1]<0) || (v[0]>1000 && v[0]<1050 && v[1]>-50 && v[1]<650) || (v[0]>-50 && v[0]<1050 && v[1]>600 && v[1]<650))
                            {
                                int row = v[0]+50;
                                int col = v[1]+50;
                                if((row+col)%2==1)
                                {
                                    Diffuse[0] = 66*1;
                                    Diffuse[1] = 29*1;
                                    Diffuse[2] = 20*1;
                                }
                                else
                                {
                                    Diffuse[0] = 66*2;
                                    Diffuse[1] = 29*2;
                                    Diffuse[2] = 20*2;
                                }

                                

                                Ambient[0] = 66;
                                Ambient[1] = 29;
                                Ambient[2] = 20;

                                Specular[0] = 0;
                                Specular[1] = 0;
                                Specular[2] = 0;
                            }

                        else
                        {
                            Ambient[0] = 20;
                            Ambient[1] = 20;
                            Ambient[2] = 20;
                            Diffuse[0] = 0;
                            Diffuse[1] = 0;
                            Diffuse[2] = 0;
                            Specular[0] = 0;
                            Specular[1] = 0;
                            Specular[2] = 0;
                        }
                        
                    }
                    
                }
            }
            }
            
            

            data[k] = Ambient[0]+Diffuse[0]+Specular[0];
            if(data[k]>255) data[k]=255;
            data[k+1] = Ambient[1]+Diffuse[1]+Specular[1];
            if(data[k+1]>255) data[k+1]=255;
            data[k+2] = Ambient[2]+Diffuse[2]+Specular[2];
            if(data[k+2]>255) data[k+2]=255;
            k=k+3;
            
        }

        
    }

    for (int i = 0; i < 3*width*height; i++) {
        unsigned char c = (unsigned char) (data[i]); 
        outFile.write((char*) &c, sizeof(c));
    }
        outFile.close();

    return 0;
}