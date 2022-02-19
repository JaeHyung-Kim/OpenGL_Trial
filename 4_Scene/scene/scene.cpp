// sudo apt-get install g++ 
// sudo apt-get install freeglut3-dev 
// sudo apt-get install mesa-utils
// sudo apt install libglm-dev


#include <iostream>
#include <GL/glut.h>
#include <sstream>
#include <math.h>
#include <GL/glut.h>
#include <glm/glm.hpp>
#include <fstream>
#include <cstring>
#include <vector>
using namespace std;
using namespace glm;


static unsigned int width = 1000;
static unsigned int height = 1000;

static bool mouseRotatePressed = false;
static bool mouseMovePressed   = false;
static bool mouseZoomPressed   = false;
static bool mouseDollyPressed   = false;
static bool seekPlz   = false;

static int	lastX = 0;
static float theta=0.0, lastTheta=0.0;
static int	lastY = 0;
static float beta=0.0, lastBeta=0.0;

static int	lastX_t = 0;
static float theta_t=0.0, lastTheta_t=0.0;
static int	lastY_t = 0;
static float beta_t=0.0, lastBeta_t=0.0;

static int	lastY_z = 0;
static float beta_z=0.0, lastBeta_z=0.0;

static int	lastY_d = 0;
static float beta_d=0.0, lastBeta_d=0.0;

static float x_0 = -220.0;
static float y_0 = -220.0;
static float z_0 = 170.0;

static float origin_x = 500.0;
static float origin_y = 300.0;
static float origin_z = 0.0;

static float camx = 0.0;
static float camy = 0.0;
static float camz = 1.0;

static float viewing_angle = 50.0;

static bool fullScreen = false;

static int rotationsign = 1;

static float rad;

static int num_in_section = 10; // n deung bun of interpolating sections in one gan gyeok
static int num_in_points = 20; // points between two control points

static double pi = 3.14159265;

struct quaternion
{
    float w, x, y, z;

    static quaternion ArbitraryAxis(float angle, float x, float y, float z)
    {
        float xaxis, yaxis, zaxis;
        xaxis = x / (sqrt(x*x+y*y+z*z));
        yaxis = y / (sqrt(x*x+y*y+z*z));
        zaxis = z / (sqrt(x*x+y*y+z*z));
        quaternion q;
        q.w = cos(angle/2.0);
        q.x = xaxis*sin(angle/2.0);
        q.y = yaxis*sin(angle/2.0);
        q.z = zaxis*sin(angle/2.0);
        return q;

    }

    static quaternion Euler2Q(float roll, float pitch)
    {
        quaternion q;
        float cosr = cos(roll/2.0);
        float sinr = sin(roll/2.0);
        float cosp = cos(pitch/2.0);
        float sinp = sin(pitch/2.0);
        q.w = cosr*cosp;
        q.x = cosp*sinr;
        q.y = sinr*sinp;
        q.z = cosr*sinp;
        return q;
    }

    static quaternion X2Q(float X)
    {
        // axis is y axis, local coord at camera, (camx, camy, camz) ->normalize
        float xaxis, yaxis, zaxis;
        xaxis = camx / (sqrt(camx*camx+camy*camy+camz*camz));
        yaxis = camy / (sqrt(camx*camx+camy*camy+camz*camz));
        zaxis = camz / (sqrt(camx*camx+camy*camy+camz*camz));
        quaternion q;
        q.w = cos(X/2);
        q.x = xaxis*sin(X/2);
        q.y = yaxis*sin(X/2);
        q.z = zaxis*sin(X/2);
        return q;
    }

    static quaternion Y2Q(float Y)
    {
        // axis is x axis, local coord at camera, (camx, camy, camz) X (x_0, y_0, z_0) => normalized
        float xaxis, yaxis, zaxis, xtmp, ytmp, ztmp;
        xtmp = camy*(z_0-origin_z) - camz*(y_0-origin_y);
        ytmp = camz*(x_0-origin_x) - camx*(z_0-origin_z);
        ztmp = camx*(y_0-origin_y) - camy*(x_0-origin_x);

        xaxis = xtmp / (sqrt(xtmp*xtmp+ytmp*ytmp+ztmp*ztmp));
        yaxis = ytmp / (sqrt(xtmp*xtmp+ytmp*ytmp+ztmp*ztmp));
        zaxis = ztmp / (sqrt(xtmp*xtmp+ytmp*ytmp+ztmp*ztmp));

        quaternion q;
        q.w = cos(Y/2);
        q.x = xaxis*sin(Y/2);
        q.y = yaxis*sin(Y/2);
        q.z = zaxis*sin(Y/2);
        return q;
    }


    static quaternion Mul(quaternion a, quaternion b)
    {
        quaternion q;
        q.w = a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z;
        q.x = a.w*b.x + b.w*a.x + a.y*b.z - a.z*b.y;
        q.y = a.w*b.y + b.w*a.y + a.z*b.x - a.x*b.z;
        q.z = a.w*b.z + b.w*a.z + a.x*b.y - a.y*b.x;
        return q;
    }

    static quaternion Inv(quaternion a)
    {
        quaternion q;
        q.w = a.w;
        q.x = -a.x;
        q.y = -a.y;
        q.z = -a.z;
        return q;
    }

    static quaternion Pos(float X, float Y, float Z)
    {
        quaternion q;
        q.x = X;
        q.y = Y;
        q.z = Z;
        q.w = 0.0;
        return q;
    }

    static quaternion Translation(quaternion pos, float X, float Y)
    {
        float xaxis, yaxis, zaxis, xtmp, ytmp, ztmp;

        // x axis (local)
        xtmp = camy*(z_0-origin_z) - camz*(y_0-origin_y);
        ytmp = camz*(x_0-origin_x) - camx*(z_0-origin_z);
        ztmp = camx*(y_0-origin_y) - camy*(x_0-origin_x);

        xaxis = xtmp / (sqrt(xtmp*xtmp+ytmp*ytmp+ztmp*ztmp));
        yaxis = ytmp / (sqrt(xtmp*xtmp+ytmp*ytmp+ztmp*ztmp));
        zaxis = ztmp / (sqrt(xtmp*xtmp+ytmp*ytmp+ztmp*ztmp));

        pos.x += xaxis*X;
        pos.y += yaxis*X;
        pos.z += zaxis*X;

        // y axis (local)
        xaxis = camx / (sqrt(camx*camx+camy*camy+camz*camz));
        yaxis = camy / (sqrt(camx*camx+camy*camy+camz*camz));
        zaxis = camz / (sqrt(camx*camx+camy*camy+camz*camz));

        pos.x += xaxis*Y;
        pos.y += yaxis*Y;
        pos.z += zaxis*Y;

        return pos;
    }

    static quaternion Dolly(quaternion pos, float Z)
    {
        float xtmp, ytmp, ztmp, xaxis, yaxis, zaxis;

        xtmp = (x_0-origin_x);
        ytmp = (y_0-origin_y);
        ztmp = (z_0-origin_z);

        xaxis = xtmp / sqrt(xtmp*xtmp+ytmp*ytmp+ztmp*ztmp);
        yaxis = ytmp / sqrt(xtmp*xtmp+ytmp*ytmp+ztmp*ztmp);
        zaxis = ztmp / sqrt(xtmp*xtmp+ytmp*ytmp+ztmp*ztmp);
        
        pos.x += xaxis*Z;
        pos.y += yaxis*Z;
        pos.z += zaxis*Z;

        return pos;
    }

};

void SQT(double scale, dvec4 rotate, dvec3 posit)
{
    glTranslatef(posit[0], posit[1], posit[2]);
    glRotatef(rotate[0], rotate[1], rotate[2], rotate[3]);
    glScalef(scale, scale, scale);
    
    
}

void sweptSurface(string filename)
{
    ifstream ifs;
    ifs.open((filename)+".txt"); rotationsign = -1;
    // ifs.open("coke_bottle.txt"); rotationsign = 1;
    // ifs.open("joystick.txt"); rotationsign = 1;

    if (!ifs.is_open())
	{
		std::cerr << "Error occured in reading txt file!" << std::endl;
        return;
	}
    string curve_type;
    ifs >> curve_type;    

    bool isBspline = false;
    if(curve_type == "BSPLINE")
    {
        isBspline = true;
    }

    int num_section;
    ifs >> num_section;

    int num_points;
    ifs >> num_points;


    dvec2 ctrl[50][50]; // coord of [i th section][j th ctrl point]
    double scale[50]; // scale of [i th section]
    dvec4 rotate[50];
    dvec3 posit[50]; 

    for(int i=0; i<num_section; i++)
    {
        for(int j=0; j<num_points; j++)
        {
            ifs >> ctrl[i][j][0];
            ifs >> ctrl[i][j][1];
        }
        ifs >> scale[i];
        ifs >> rotate[i][0];
        ifs >> rotate[i][1];
        ifs >> rotate[i][2];
        ifs >> rotate[i][3];

        rotate[i][0] = rotationsign * rotate[i][0]; // rotation sign check
        if(abs(rotate[i][0]) < 0.01)
        {
            rotate[i][0] = 0;
            rotate[i][1] = 0;
            rotate[i][2] = 0;
            rotate[i][3] = 1;
        }
        ifs >> posit[i][0];
        ifs >> posit[i][1];
        ifs >> posit[i][2];

    }

    // overall plane # = (num_section-1) * (num_in_section-1) + 1
    // [i] -> i / num_in_section   th section   i % num_in_section   th interpolated section

    dvec2 in_ctrl[500][50]; // i th interpolated section j th control point
    double in_scale[500];
    dvec4 in_rotate[500];
    dvec3 in_posit[500];

    for(int i=0; i<(num_section-1) * (num_in_section) + 1; i++)
    {
        // i/num_in_section th plane ~ i/num_in_section + 1 th plane interpolate
        // t = i%num_in_section / num_in_section
        int n = i / num_in_section;
        if(i%num_in_section == 0)
        {
            in_scale[i] = scale[n];
            in_rotate[i] = rotate[n];
            in_posit[i] = posit[n];
            for(int j=0; j<num_points; j++)
            {
                in_ctrl[i][j] = ctrl[n][j];
            }
        }
        else
        {
            float t = ((float)(i%num_in_section))/((float)num_in_section);
            in_scale[i] = (-0.5*t+t*t-0.5*t*t*t)*scale[n-1] + (1.0-2.5*t*t+1.5*t*t*t)*scale[n] + (0.5*t+2.0*t*t-1.5*t*t*t)*scale[n+1] + (-0.5*t*t+0.5*t*t*t)*scale[n+2];
            in_rotate[i] = (-0.5*t+t*t-0.5*t*t*t)*rotate[n-1] + (1.0-2.5*t*t+1.5*t*t*t)*rotate[n] + (0.5*t+2.0*t*t-1.5*t*t*t)*rotate[n+1] + (-0.5*t*t+0.5*t*t*t)*rotate[n+2];
            in_posit[i] = (-0.5*t+t*t-0.5*t*t*t)*posit[n-1] + (1.0-2.5*t*t+1.5*t*t*t)*posit[n] + (0.5*t+2.0*t*t-1.5*t*t*t)*posit[n+1] + (-0.5*t*t+0.5*t*t*t)*posit[n+2];
            for(int j=0; j<num_points; j++)
            {
                if(n==0)
                {
                    // in_ctrl[i][j] = (-0.5*t+t*t-0.5*t*t*t)*ctrl[n][j] + (1.0-2.5*t*t+1.5*t*t*t)*ctrl[n][j] + (0.5*t+2.0*t*t-1.5*t*t*t)*ctrl[n+1][j] + (-0.5*t*t+0.5*t*t*t)*ctrl[n+2][j];
                
                }
                else
                {
                    in_ctrl[i][j] = (-0.5*t+t*t-0.5*t*t*t)*ctrl[n-1][j] + (1.0-2.5*t*t+1.5*t*t*t)*ctrl[n][j] + (0.5*t+2.0*t*t-1.5*t*t*t)*ctrl[n+1][j] + (-0.5*t*t+0.5*t*t*t)*ctrl[n+2][j];
                
                }
            }
        }
        in_rotate[i] = in_rotate[i];
    }


    dvec2 in_points[500][500]; // i th interpolated section j th interpolated point

    for(int i=0; i<(num_section) * (num_in_section-1) + 1; i++)
    {
        for(int j=0; j<num_points; j++)
        {
            for(int k=0; k<num_in_points; k++)
            { // overall 0 ~ (num_points*num_in_points - 1) amounts of point in one section
                float t = (float)k / float(num_in_points);
                if(isBspline)
                {
                    in_points[i][k + j*num_in_points][0] = (1.0-t)*(1.0-t)*(1.0-t)*in_ctrl[i][j][0]/6.0 + (3.0*t*t*t-6.0*t*t+4.0)*in_ctrl[i][(j+1)%num_points][0]/6.0 + (-3.0*t*t*t+3.0*t*t+3.0*t+1.0)*in_ctrl[i][(j+2)%num_points][0]/6.0+t*t*t*in_ctrl[i][(j+3)%num_points][0]/6.0;
                    in_points[i][k + j*num_in_points][1] = (1.0-t)*(1.0-t)*(1.0-t)*in_ctrl[i][j][1]/6.0 + (3.0*t*t*t-6.0*t*t+4.0)*in_ctrl[i][(j+1)%num_points][1]/6.0 + (-3.0*t*t*t+3.0*t*t+3.0*t+1.0)*in_ctrl[i][(j+2)%num_points][1]/6.0+t*t*t*in_ctrl[i][(j+3)%num_points][1]/6.0;
                    
                }
                else
                {
                    in_points[i][k + j*num_in_points][0] = (-0.5*t+t*t-0.5*t*t*t)*in_ctrl[i][j][0] + (1.0-2.5*t*t+1.5*t*t*t)*in_ctrl[i][(j+1)%num_points][0] + (0.5*t+2.0*t*t-1.5*t*t*t)*in_ctrl[i][(j+2)%num_points][0] + (-0.5*t*t+0.5*t*t*t)*in_ctrl[i][(j+3)%num_points][0];
                    in_points[i][k + j*num_in_points][1] = (-0.5*t+t*t-0.5*t*t*t)*in_ctrl[i][j][1] + (1.0-2.5*t*t+1.5*t*t*t)*in_ctrl[i][(j+1)%num_points][1] + (0.5*t+2.0*t*t-1.5*t*t*t)*in_ctrl[i][(j+2)%num_points][1] + (-0.5*t*t+0.5*t*t*t)*in_ctrl[i][(j+3)%num_points][1];
                }
            }
        }
    
    }    

    for(int i=num_in_section; i<(num_section) * (num_in_section-1)-1; i++)
    {
        quaternion rot_i = quaternion::ArbitraryAxis(in_rotate[i][0], in_rotate[i][1], in_rotate[i][2], in_rotate[i][3]);
        quaternion inv_rot_i = quaternion::Inv(rot_i);

        quaternion rot_i1 = quaternion::ArbitraryAxis(in_rotate[i+1][0], in_rotate[i+1][1], in_rotate[i+1][2], in_rotate[i+1][3]);
        quaternion inv_rot_i1 = quaternion::Inv(rot_i1);

        // glBegin(GL_LINE_LOOP);
        
        for(int j=0; j<num_points*num_in_points; j++)
        {
            quaternion qij;
            qij.w = 0.0;
            qij.x = in_scale[i] * in_points[i][j][0];
            qij.y = 0.0;
            qij.z = in_scale[i] * in_points[i][j][1];

            quaternion ux_ij = quaternion::Mul(quaternion::Mul(rot_i, qij), inv_rot_i);
            GLfloat xij = in_posit[i][0] + ux_ij.x;
            GLfloat yij = in_posit[i][1] + ux_ij.y;
            GLfloat zij = in_posit[i][2] + ux_ij.z;

            quaternion qij1;
            int j_1 = (j+1)%(num_points*num_in_points);
            qij1.w = 0.0;
            qij1.x = in_scale[i] * in_points[i][j_1][0];
            qij1.y = 0.0;
            qij1.z = in_scale[i] * in_points[i][j_1][1];

            quaternion ux_ij1 = quaternion::Mul(quaternion::Mul(rot_i, qij1), inv_rot_i);
            GLfloat xij1 = in_posit[i][0] + ux_ij1.x;
            GLfloat yij1 = in_posit[i][1] + ux_ij1.y;
            GLfloat zij1 = in_posit[i][2] + ux_ij1.z;

            quaternion qi1j;
            qi1j.w = 0.0;
            qi1j.x = in_scale[i+1] * in_points[i+1][j][0];
            qi1j.y = 0.0;
            qi1j.z = in_scale[i+1] * in_points[i+1][j][1];

            quaternion ux_i1j = quaternion::Mul(quaternion::Mul(rot_i1, qi1j), inv_rot_i1);
            GLfloat xi1j = in_posit[i+1][0] + ux_i1j.x;
            GLfloat yi1j = in_posit[i+1][1] + ux_i1j.y;
            GLfloat zi1j = in_posit[i+1][2] + ux_i1j.z;

            quaternion qi1j1;
            qi1j1.w = 0.0;
            qi1j1.x = in_scale[i+1] * in_points[i+1][j_1][0];
            qi1j1.y = 0.0;
            qi1j1.z = in_scale[i+1] * in_points[i+1][j_1][1];

            quaternion ux_i1j1 = quaternion::Mul(quaternion::Mul(rot_i1, qi1j1), inv_rot_i1);
            GLfloat xi1j1 = in_posit[i+1][0] + ux_i1j1.x;
            GLfloat yi1j1 = in_posit[i+1][1] + ux_i1j1.y;
            GLfloat zi1j1 = in_posit[i+1][2] + ux_i1j1.z;


            glBegin(GL_TRIANGLES);
            glNormal3f(ux_ij.x, ux_ij.y, ux_ij.z);
            glVertex3f(xij, yij, zij);
            glVertex3f(xij1, yij1, zij1);
            glVertex3f(xi1j, yi1j, zi1j);
            glEnd();

            glBegin(GL_TRIANGLES);
            glNormal3f(ux_i1j.x, ux_i1j.y, ux_i1j.z);
            glVertex3f(xi1j1, yi1j1, zi1j1);
            glVertex3f(xi1j, yi1j, zi1j);
            glVertex3f(xij1, yij1, zij1);
            glEnd();

        }

    }
}

void floor()
{   
    glPushMatrix();
    float blue[13] = { 0.04725, 0.0995, 0.5745, 1.0,      /* ambient */
                   0.05164, 0.00648, 0.82648, 1.0,    /* diffuse */
                   0.228281, 0.255802, 0.766065, 1.0, /* specular */
                   30.0                               /* shininess */
    };
    glMaterialfv( GL_FRONT_AND_BACK, GL_AMBIENT, blue );
    glMaterialfv( GL_FRONT_AND_BACK, GL_DIFFUSE, &blue[4] );
    glMaterialfv( GL_FRONT_AND_BACK, GL_SPECULAR, &blue[8] );
    glMaterialf( GL_FRONT_AND_BACK, GL_SHININESS, blue[12] );
    float normalArray[3] = { 0.0, 0.0, 1.0 };
    glNormal3fv(normalArray);
    glBegin(GL_POLYGON);
    glVertex3f(0, 0, 0);
    glVertex3f(1000, 0, 0);
    glVertex3f(1000, 600, 0);
    glVertex3f(0, 600, 0);
    glVertex3f(0, 0, 0);
    glEnd();
    glPopMatrix();
}

void wood()
{   
    glPushMatrix();
    float wood[13] = { 0.2588, 0.1137, 0.1, 1.0,      /* ambient */
                   0.2588, 0.1137, 0.1, 1.0,    /* diffuse */
                   0.2588, 0.1137, 0.1, 1.0, /* specular */
                   80.0                               /* shininess */
    };
    glMaterialfv( GL_FRONT_AND_BACK, GL_AMBIENT, wood );
    glMaterialfv( GL_FRONT_AND_BACK, GL_DIFFUSE, &wood[4] );
    glMaterialfv( GL_FRONT_AND_BACK, GL_SPECULAR, &wood[8] );
    glMaterialf( GL_FRONT_AND_BACK, GL_SHININESS, wood[12] );
    float normalArray[3] = { 0.0, 0.0, 1.0 };
    glNormal3fv(normalArray);
    glBegin(GL_QUADS);

    glVertex3f(0, 0, 10);
    glVertex3f(0, -50, 10);
    glVertex3f(1050, -50, 10);
    glVertex3f(1050, 0, 10);

    glVertex3f(1000, 0, 10);
    glVertex3f(1050, 0, 10);
    glVertex3f(1050, 650, 10);
    glVertex3f(1000, 650, 10);

    glVertex3f(1000, 600, 10);
    glVertex3f(1000, 650, 10);
    glVertex3f(-50, 650, 10);
    glVertex3f(-50, 600, 10);

    glVertex3f(0, 600, 10);
    glVertex3f(-50, 600, 10);
    glVertex3f(-50, -50, 10);
    glVertex3f(0, -50, 10);

    float wood2[13] = { 0.05, 0.05, 0.4, 1.0,      /* ambient */
                   0.05, 0.05, 0.4, 1.0,    /* diffuse */
                   0.0, 0.0, 0.2, 1.0, /* specular */
                   20.0                               /* shininess */
    };
    glMaterialfv( GL_FRONT_AND_BACK, GL_AMBIENT, wood2 );
    glMaterialfv( GL_FRONT_AND_BACK, GL_DIFFUSE, &wood2[4] );
    glMaterialfv( GL_FRONT_AND_BACK, GL_SPECULAR, &wood2[8] );
    glMaterialf( GL_FRONT_AND_BACK, GL_SHININESS, wood2[12] );

    float normalArray1[3] = { 0.0, 1.0, 0.0 };
    glNormal3fv(normalArray1);
    glVertex3f(0, 0, 0);
    glVertex3f(0, 0, 10);
    glVertex3f(1000, 0, 10);
    glVertex3f(1000, 0, 0);

    float normalArray2[3] = { -1.0, 0.0, 0.0 };
    glNormal3fv(normalArray2);
    glVertex3f(1000, 600, 0);
    glVertex3f(1000, 600, 10);
    glVertex3f(1000, 0, 10);
    glVertex3f(1000, 0, 0);

    float normalArray3[3] = { 0.0, -1.0, 0.0 };
    glNormal3fv(normalArray3);
    glVertex3f(1000, 600, 0);
    glVertex3f(1000, 600, 10);
    glVertex3f(0, 600, 10);
    glVertex3f(0, 600, 0);

    float normalArray4[3] = { 1.0, .0, 0.0 };
    glNormal3fv(normalArray4);
    glVertex3f(0, 0, 0);
    glVertex3f(0, 0, 10);
    glVertex3f(0, 600, 10);
    glVertex3f(0, 600, 0);


    glEnd();


    glPopMatrix();
}

void bulb()
{
    glPushMatrix();
    float gold[13] = { 0.24725, 0.1995, 0.0745, 1.0,      /* ambient */
                   0.75164, 0.60648, 0.22648, 1.0,    /* diffuse */
                   0.628281, 0.555802, 0.366065, 1.0, /* specular */
                   50.0                               /* shininess */
    };
    glMaterialfv( GL_FRONT_AND_BACK, GL_AMBIENT, gold );
    glMaterialfv( GL_FRONT_AND_BACK, GL_DIFFUSE, &gold[4] );
    glMaterialfv( GL_FRONT_AND_BACK, GL_SPECULAR, &gold[8] );
    glMaterialf( GL_FRONT_AND_BACK, GL_SHININESS, gold[12] );  
    glRotatef(90, 1, 0, 0);
    sweptSurface("bulb");
    glPopMatrix();
}

void cue0()
{
    float gol[13] = { 0, 0, 0, 1.0,      /* ambient */
                   0, 0, 0, 1.0,    /* diffuse */
                   0,0,0, 1.0, /* specular */
                   10.0                               /* shininess */
    };
    glMaterialfv( GL_FRONT_AND_BACK, GL_AMBIENT, gol );
    glMaterialfv( GL_FRONT_AND_BACK, GL_DIFFUSE, &gol[4] );
    glMaterialfv( GL_FRONT_AND_BACK, GL_SPECULAR, &gol[8] );
    glMaterialf( GL_FRONT_AND_BACK, GL_SHININESS, gol[12] );
    sweptSurface("cue0"); 
}

void cue1()
{
    num_in_section = 50;
    // 0.7843 0.3921 0.1961
    float gold[13] = { 0.9843, 0.6921, 0.5961, 1.0,      /* ambient */
                   0.05164, 0.00648, 0.02648, 1.0,    /* diffuse */
                   0.000007843, 0.000003921, 0.000001961, 1.0, /* specular */
                   10.0                               /* shininess */
    };
    glMaterialfv( GL_FRONT_AND_BACK, GL_AMBIENT, gold );
    glMaterialfv( GL_FRONT_AND_BACK, GL_DIFFUSE, &gold[4] );
    glMaterialfv( GL_FRONT_AND_BACK, GL_SPECULAR, &gold[8] );
    glMaterialf( GL_FRONT_AND_BACK, GL_SHININESS, gold[12] );
    sweptSurface("cue1");  
    num_in_section = 10;

}

void cue2()
{
    num_in_section = 50;
     float black[13] = { 0.5843, 0.3921, 0.2961, 1.0,      /* ambient */
                   0.05164, 0.00648, 0.02648, 1.0,    /* diffuse */
                   0.000007843, 0.000003921, 0.000001961, 1.0, /* specular */
                   10.0                               /* shininess */
    };
    glMaterialfv( GL_FRONT_AND_BACK, GL_AMBIENT, black );
    glMaterialfv( GL_FRONT_AND_BACK, GL_DIFFUSE, &black[4] );
    glMaterialfv( GL_FRONT_AND_BACK, GL_SPECULAR, &black[8] );
    glMaterialf( GL_FRONT_AND_BACK, GL_SHININESS, black[12] );  
    sweptSurface("cue2");  
    num_in_section = 10;
}

vector<string> split(string input, char delimiter) {
    vector<string> answer;
    stringstream ss(input);
    string temp;
 
    while (getline(ss, temp, delimiter)) {
        answer.push_back(temp);
    }
 
    return answer;
}

void objLoad(string name, float ratio)
{

    fvec3 vertices[100000];
    int index_v = 1;
    fvec3 normals[100000];
    int index_vn = 1;
    int index_f = 0;

    fvec3 faces[100000];
    ifstream ifs(name + ".obj");

    if (!ifs.is_open())
	{
		std::cerr << "Error occured in reading obj file!" << std::endl;
        return;
	}
    string buffer;

    glBegin(GL_QUADS);

    while (ifs.peek() != EOF) {
        // std::getline은 입력 스트림에서 string으로 한 줄을 읽습니다.
        ifs >> buffer;
        if(buffer == "v")
        {
            fvec3 abc;
            ifs >> abc[0] >> abc[1] >> abc[2];
            vertices[index_v] = abc;
            index_v ++;
        }

        if(buffer == "vn")
        {
            fvec3 abc;
            ifs >> abc[0] >> abc[1] >> abc[2];
            normals[index_vn] = abc;
            index_vn ++;
        }

        if(buffer == "f")
        {
            
            string abc;
            ifs >> abc;
            vector<string> result = split(abc, '/');
            int vi = stoi(result[0]);
            int ni = stoi(result[2]);
            glVertex3f(vertices[vi][0]*ratio, vertices[vi][1]*ratio, vertices[vi][2]*ratio);
            glNormal3f(normals[ni][0]*ratio, normals[ni][1]*ratio, normals[ni][2]*ratio);

            ifs >> abc;
            result = split(abc, '/');
            vi = stoi(result[0]);
            ni = stoi(result[2]);
            glVertex3f(vertices[vi][0]*ratio, vertices[vi][1]*ratio, vertices[vi][2]*ratio);
            glNormal3f(normals[ni][0]*ratio, normals[ni][1]*ratio, normals[ni][2]*ratio);

            ifs >> abc;
            result = split(abc, '/');
            vi = stoi(result[0]);
            ni = stoi(result[2]);
            glVertex3f(vertices[vi][0]*ratio, vertices[vi][1]*ratio, vertices[vi][2]*ratio);
            glNormal3f(normals[ni][0]*ratio, normals[ni][1]*ratio, normals[ni][2]*ratio);

            ifs >> abc;
            result = split(abc, '/');
            vi = stoi(result[0]);
            ni = stoi(result[2]);
            glVertex3f(vertices[vi][0]*ratio, vertices[vi][1]*ratio, vertices[vi][2]*ratio);
            glNormal3f(normals[ni][0]*ratio, normals[ni][1]*ratio, normals[ni][2]*ratio);

            index_f ++;

        }
    }
    glEnd();
}

void hand()
{
    float gold[13] = { 0.8915*0.6, 0.6547*0.6, 0.5664*0.6, 1.0,      /* ambient */
                   0.8915*0.2, 0.6547*0.2, 0.5664*0.2, 1.0,    /* diffuse */
                   0.8515*0.2, 0.5547*0.2, 0.5664*0.2, 1.0, /* specular */
                   0.0                               /* shininess */
    };
    glMaterialfv( GL_FRONT_AND_BACK, GL_AMBIENT, gold );
    glMaterialfv( GL_FRONT_AND_BACK, GL_DIFFUSE, &gold[4] );
    glMaterialfv( GL_FRONT_AND_BACK, GL_SPECULAR, &gold[8] );
    glMaterialf( GL_FRONT_AND_BACK, GL_SHININESS, gold[12] );
    objLoad("hand", 1.0);
}

void chalk()
{
    float gold[13] = { 0.0543, 0.2221, 0.2261, 1.0,      /* ambient */
                   0.05164, 0.1648, 0.1648, 1.0,    /* diffuse */
                   0.0843, 0.36921, 0.36961, 1.0, /* specular */
                   0.0                               /* shininess */
    };
    glMaterialfv( GL_FRONT_AND_BACK, GL_AMBIENT, gold );
    glMaterialfv( GL_FRONT_AND_BACK, GL_DIFFUSE, &gold[4] );
    glMaterialfv( GL_FRONT_AND_BACK, GL_SPECULAR, &gold[8] );
    glMaterialf( GL_FRONT_AND_BACK, GL_SHININESS, gold[12] );
    
    objLoad("chalk", 3.5);
}

void sphere(float radius)
{
    glTranslatef(0, 0, radius);
    fvec3 vertices[100][100];
    float theta, alpha, R;
    float sx, sy, sz;
    
    for(int i=0; i<=50; i++)
    {
        theta = i*pi/50.0-pi/2.0;
        sz = radius * sin(theta);
        R = radius * cos(theta);
        for(int j=0; j<=50; j++)
        {
            alpha = 2.0*pi*j/50.0;
            sx = R*cos(alpha);
            sy = R*sin(alpha);
            vertices[i][j][0] = sx;
            vertices[i][j][1] = sy;
            vertices[i][j][2] = sz;
        }
    }
    glBegin(GL_QUADS);
    for(int i=0; i<=50; i++)
    {
        for(int j=0; j<=50; j++)
        {
            glNormal3f(vertices[i][j][0], vertices[i][j][1], vertices[i][j][2]);
            glVertex3f(vertices[i][j][0], vertices[i][j][1], vertices[i][j][2]);
            glVertex3f(vertices[i][j+1][0], vertices[i][j+1][1], vertices[i][j+1][2]);
            glVertex3f(vertices[i+1][j+1][0], vertices[i+1][j+1][1], vertices[i+1][j+1][2]);
            glVertex3f(vertices[i+1][j][0], vertices[i+1][j][1], vertices[i+1][j][2]);
        }

    }
    glEnd();
}

void yellow()
{
    float gold[13] = { 1.0*0.5, 0.8*0.5, 0, 1.0,      /* ambient */
                   1.0*0.15, 0.8*0.15, 0, 1.0,    /* diffuse */
                   1.0*0.15, 0.8*0.15, 0, 1.0, /* specular */
                   20.0                               /* shininess */
    };
    glMaterialfv( GL_FRONT_AND_BACK, GL_AMBIENT, gold );
    glMaterialfv( GL_FRONT_AND_BACK, GL_DIFFUSE, &gold[4] );
    glMaterialfv( GL_FRONT_AND_BACK, GL_SPECULAR, &gold[8] );
    glMaterialf( GL_FRONT_AND_BACK, GL_SHININESS, gold[12] );
}

void red()
{
    float gold[13] = { 1.0 * 0.4, 0, 0, 1.0,      /* ambient */
                   1.0 * 0.1, 0, 0, 1.0,    /* diffuse */
                   1.0 * 0.1, 0, 0, 1.0, /* specular */
                   20.0                               /* shininess */
    };
    glMaterialfv( GL_FRONT_AND_BACK, GL_AMBIENT, gold );
    glMaterialfv( GL_FRONT_AND_BACK, GL_DIFFUSE, &gold[4] );
    glMaterialfv( GL_FRONT_AND_BACK, GL_SPECULAR, &gold[8] );
    glMaterialf( GL_FRONT_AND_BACK, GL_SHININESS, gold[12] );
}

void white()
{
    float gold[13] = { 1.0 * 0.4, 1.0 * 0.4, 1.0 * 0.4, 1.0,      /* ambient */
                   1.0 * 0.1, 1.0 * 0.1, 1.0 * 0.1, 1.0,    /* diffuse */
                   1.0 * 0.1, 1.0 * 0.1, 1.0 * 0.1, 1.0, /* specular */
                   20.0                               /* shininess */
    };
    glMaterialfv( GL_FRONT_AND_BACK, GL_AMBIENT, gold );
    glMaterialfv( GL_FRONT_AND_BACK, GL_DIFFUSE, &gold[4] );
    glMaterialfv( GL_FRONT_AND_BACK, GL_SPECULAR, &gold[8] );
    glMaterialf( GL_FRONT_AND_BACK, GL_SHININESS, gold[12] );
}

void drawSomething()
{


    glClear (GL_COLOR_BUFFER_BIT);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();
    floor();
    wood();
    glPushMatrix();
    glTranslatef(100, 100, 200);
    bulb();
    glTranslatef(200, 0, 0);
    bulb();
    glTranslatef(200, 0, 0);
    bulb();
    glPopMatrix();

    glPushMatrix();
    glTranslatef(150, -10, 0);
    glRotatef(55, 0, 0, 1);
    glTranslatef(200, 150, 30);
    glRotatef(90, 0, 0, 1);
    cue0();
    cue1();
    cue2();
    glPopMatrix();
    
    glPushMatrix();
    glTranslatef(150, -10, 0);
    glRotatef(55, 0, 0, 1);
    glTranslatef(100, 170, 22);
    glRotatef(0, 0, 0, 1);
    hand();
    glPopMatrix();

    glPushMatrix();
    glTranslatef(-30, 70, 10);
    chalk();
    glPopMatrix();

    glPushMatrix();
    glTranslatef(25, 60, 0);
    yellow();
    sphere(12.0);
    glPopMatrix();

    glPushMatrix();
    glTranslatef(60, 40, 0);
    red();
    sphere(12.0);
    glPopMatrix();

    glPushMatrix();
    glTranslatef(170, 150, 0);
    white();
    sphere(12.0);
    glPopMatrix();



    

}

void reshape(int w, int h)
{
    width = glutGet(GLUT_WINDOW_WIDTH);
    height = glutGet(GLUT_WINDOW_HEIGHT);

    glViewport(0, 0, w, h );

	glMatrixMode( GL_PROJECTION );
	glLoadIdentity();
	float aspectRatio = (float)w/(float)h;
	gluPerspective( viewing_angle /*field of view angle*/,
					aspectRatio,
					10.0 /*near clipping plane*/,
					2000.0 /* far clipping plane */ );

	gluLookAt( x_0,y_0,z_0, origin_x,origin_y,origin_z, camx,camy,camz );
	glMatrixMode( GL_MODELVIEW );
}


void display()
{
    glLoadIdentity();
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    drawSomething();
    glFlush();
    glutSwapBuffers();
}

void keyboardCB(unsigned char keyPressed, int x, int y)
{
    switch (keyPressed) {
    case 'f':
        if (fullScreen == true) {
            glutReshapeWindow(width,height);
            fullScreen = false;
        } else {
            glutFullScreen();
            fullScreen = true;
        }
        break;
    case 'q':
        exit(0);
        break;
    case 'a':
        // press a to show all figure

        // put origin to 0, 0, 0
        origin_x = 0.0;
        origin_y = 0.0;
        origin_z = 0.0;

        // put camera on the same way
        x_0 = x_0 - origin_x;
        y_0 = y_0 - origin_y;
        z_0 = z_0 - origin_z;

        // maintain camx camy camz

        // normalize camera vector
        // put camera to the appropriate position
        
        rad = sqrt(x_0*x_0+y_0*y_0+z_0*z_0);

        x_0 = 3.0 * x_0 / rad;
        y_0 = 3.0 * y_0 / rad;
        z_0 = 3.0 * z_0 / rad;

        reshape(width, height);
        break;
    case 's':
        // press s to seek point
        seekPlz = true;
        break;
    }
    glutPostRedisplay();
}

void mouseCB(int button, int state, int x, int y) {
    if (state == GLUT_UP) {
        mouseMovePressed   = false;
        mouseRotatePressed = false;
        mouseZoomPressed   = false;
        mouseDollyPressed  = false;
    } else {
        if (button==GLUT_LEFT_BUTTON && GLUT_ACTIVE_SHIFT==glutGetModifiers())
        {
			// do something here
			//std::cout << "translate click" << std::endl;

            lastX_t = x;
			lastTheta_t = theta_t;
            lastY_t = y;
			lastBeta_t = beta_t;

            mouseMovePressed   = true;
            mouseRotatePressed = false;
            mouseZoomPressed   = false;
            mouseDollyPressed  = false;
        }
        else if (button==GLUT_LEFT_BUTTON && GLUT_ACTIVE_CTRL==glutGetModifiers())
        {
			// do something here
			//std::cout << "zoom click" << std::endl;

            lastY_z = y;
			lastBeta_z = beta_z;

            mouseMovePressed   = false;
            mouseRotatePressed = false;
            mouseZoomPressed   = true;
            mouseDollyPressed  = false;
        }
        else if (button==GLUT_LEFT_BUTTON && GLUT_ACTIVE_ALT==glutGetModifiers())
        {

            lastY_d = y;
			lastBeta_d = beta_d;

            mouseMovePressed   = false;
            mouseRotatePressed = false;
            mouseZoomPressed   = false;
            mouseDollyPressed  = true;
        }
        else if (button==GLUT_LEFT_BUTTON)
        {
			// do something here
			//std::cout << "rotate click" << std::endl;
			lastX = x;
			lastTheta = theta;
            lastY = y;
			lastBeta = beta;

            mouseMovePressed   = false;
            mouseRotatePressed = true;
            mouseZoomPressed   = false;
            mouseDollyPressed  = false;
        }
    }
    glutPostRedisplay();
}

void motionCB(int x, int y) {
    if(seekPlz == true)
    {
        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();
        GLfloat z_f;
        glReadPixels (x, y, width, height, GL_DEPTH_COMPONENT, GL_FLOAT, &z_f);
        std::cout << z_f;
        seekPlz == false;
    }
    else if (mouseRotatePressed == true)
	{
        // do something here
		//std::cout << "rotate drag" << std::endl;

		theta = (float)(x-lastX)*0.0001;
        beta = (float)(y-lastY)*0.0001;

        quaternion rot_x = quaternion::X2Q(-theta);
        quaternion rot_y = quaternion::Y2Q(-beta);
        quaternion rot = quaternion::Mul(rot_y, rot_x);
        quaternion inv = quaternion::Inv(rot);

        quaternion pos = quaternion::Pos(x_0, y_0, z_0);
        quaternion tmp = quaternion::Mul(rot, pos);
        quaternion result = quaternion::Mul(tmp, inv);

        // x_0 y_0 z_0 direction -> 0 0 -1
        x_0 = result.x;
        y_0 = result.y;
        z_0 = result.z;

        quaternion pos2 = quaternion::Pos(camx, camy, camz);
        quaternion tmp2 = quaternion::Mul(rot, pos2);
        quaternion result2 = quaternion::Mul(tmp2, inv);

        // camx camy camz direction -> 0 1 0
        camx = result2.x;
        camy = result2.y;
        camz = result2.z;

		reshape(width, height);
	}
    else if (mouseMovePressed == true)
    {
        // to translate, press shift and drag
        //shift to x axis (local) amount of theta
        //shift to y axis (local) amount of beta 

        theta_t = -(float)(x-lastX_t)*0.01;
        beta_t = (float)(y-lastY_t)*0.01;

        quaternion pos = quaternion::Pos(x_0, y_0, z_0);
        pos = quaternion::Translation(pos, theta_t, beta_t);

        x_0 = pos.x;
        y_0 = pos.y;
        z_0 = pos.z;

        quaternion origin_pos = quaternion::Pos(origin_x, origin_y, origin_z);
        origin_pos = quaternion::Translation(origin_pos, theta_t, beta_t);

        origin_x = origin_pos.x;
        origin_y = origin_pos.y;
        origin_z = origin_pos.z;

        reshape(width, height);


    }
    else if (mouseZoomPressed == true)
    {
        // to zoom, press ctrl and drag up and down
        
        beta_z = (float)(y-lastY_z)*0.001;
        viewing_angle += beta_z;
        if(viewing_angle<1) viewing_angle = 1;
        if(viewing_angle>179) viewing_angle = 179;

        reshape(width, height);
    }
    else if (mouseDollyPressed == true)
    {
        // to dolly, press alt and drag up and down

        beta_d = (float)(y-lastY_d)*0.1;
        float dist1 = sqrt((x_0-origin_x)*(x_0-origin_x)+(y_0-origin_y)*(y_0-origin_y)+(z_0-origin_z)*(z_0-origin_z));
        quaternion pos = quaternion::Pos(x_0, y_0, z_0);
        pos = quaternion::Dolly(pos, beta_d);

        float dist2 = sqrt((x_0-origin_x)*(x_0-origin_x)+(y_0-origin_y)*(y_0-origin_y)+(z_0-origin_z)*(z_0-origin_z));
        if(dist1>=10 && dist2<10)
        {
            
        }
        else
        {
            x_0 = pos.x;
            y_0 = pos.y;
            z_0 = pos.z;
        }

        

        reshape(width, height);
    }
    glutPostRedisplay();
}

void idle() { glutPostRedisplay(); }

int main(int argc, char** argv) {
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_RGBA | GLUT_DEPTH | GLUT_DOUBLE);
    glutInitWindowSize(width, height);
    glutCreateWindow("3D viewer");
    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

    reshape(width, height);
    glClearColor(0.0, 0.1, 0.3, 1.0);

    glClearDepth(1.0f);
    glDepthFunc(GL_LESS);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);

    GLfloat AmbientLightValue[] = {0.7f, 0.7f, 0.7f, 0.0f};
    GLfloat PositionLightValue[] = { -10.0f, -10.0f, 200.0f, 0.0f };

    glLightfv( GL_LIGHT0, GL_AMBIENT, AmbientLightValue );
    glLightfv( GL_LIGHT0, GL_POSITION, PositionLightValue );

    // 100 300 300   
    
    glEnable(GL_LIGHT1);
    GLfloat Specular1[] = { 0.3f, 0.3f, 0.3f, 0.0f };
    GLfloat Position1[] = { 100.0f, 100.0f, 200.0f, 0.0f };
    glLightfv( GL_LIGHT1, GL_DIFFUSE, Specular1 ); 
    glLightfv( GL_LIGHT1, GL_POSITION, Position1 );

    glEnable(GL_LIGHT2);
    GLfloat Position2[] = { 300.0f, 100.0f, 200.0f, 0.0f };
    glLightfv( GL_LIGHT1, GL_DIFFUSE, Specular1 ); 
    glLightfv( GL_LIGHT1, GL_POSITION, Position2 );

    glEnable(GL_LIGHT3);
    GLfloat Position3[] = { 500.0f, 100.0f, 200.0f, 0.0f };
    glLightfv( GL_LIGHT1, GL_DIFFUSE, Specular1 ); 
    glLightfv( GL_LIGHT1, GL_POSITION, Position3 );
    

    glutIdleFunc(idle);
    glutDisplayFunc(display);
    glutKeyboardFunc(keyboardCB);
    glutReshapeFunc(reshape);
    glutMotionFunc(motionCB);
    glutMouseFunc(mouseCB);

    glutMainLoop();
    return 0;
}
