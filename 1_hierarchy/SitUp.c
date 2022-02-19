#include <GL/glut.h>
#include <GL/gl.h>
#include <GL/glu.h>

static int a = 0;
static int b = 0;
static int c = 0;
static int d = 0;
static float headR = 0.4;
static float headG = 0.4;
static float headB = 0.4;

static float w = 0.25;

void init(void) 
{
   glClearColor (0.0, 0.0, 0.0, 0.0);
   glShadeModel (GL_FLAT);
}

void display(void)
{
    glClear (GL_COLOR_BUFFER_BIT);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    glPushMatrix(); 
        glRotatef (b, 0.0, 0.0, -1.0);
        glTranslatef(-0.5, 0.0, 0.0);

        glPushMatrix(); // upper right arm
            glTranslatef(0.0, 0.0, w);
            glRotatef (180, 0.0, 0.0, -1.0);
                    glPushMatrix();
                        glColor3f (0.3, 0.3, 0.3);
                        glScalef(1.8, 0.5, 0.5);
                        glutSolidSphere(0.2, 50, 50);
                    glPopMatrix();
            glTranslatef(-0.35, 0.0, 0.0);
            glPushMatrix(); // lower right arm
                glRotatef (c, 0.0, 0.0, 1.0);
                glRotatef (d, 0.0, -1.0, 0.0);
                glTranslatef(-0.4, 0.0, 0.0);
                    glPushMatrix();
                        glColor3f (0.3, 0.3, 0.3);
                        glScalef(1.9, 0.4, 0.4);
                        glutSolidSphere(0.2, 50, 50);
                    glPopMatrix();
                    glTranslatef(-0.35, 0.0, 0.0);
                    // hand
                    glPushMatrix();
                        glColor3f (0.3, 0.3, 0.3);
                        glScalef(0.4, 0.4, 0.4);
                        glutSolidSphere(0.2, 50, 50);
                    glPopMatrix();
            glPopMatrix();
        glPopMatrix();
    glPopMatrix();

    glPushMatrix(); // upper right leg
        glRotatef (a, 0.0, 0.0, 1.0);
        glTranslatef(0.4, 0.0, w-0.1);
                    glPushMatrix();
                        glColor3f (0.35, 0.35, 0.35);
                        glScalef(2.0, 0.75, 0.75);
                        glutSolidSphere(0.2, 50, 50);
                    glPopMatrix();
        glPushMatrix(); // lower leg
            glTranslatef(0.4, 0.0, 0.0);
            glRotatef (-a*2.0, 0.0, 0.0, 1.0);
            glTranslatef(0.4, 0.0, 0.0);
                    glPushMatrix();
                        glColor3f (0.35, 0.35, 0.35);
                        glScalef(2.0, 0.65, 0.65);
                        glutSolidSphere(0.2, 50, 50);
                    glPopMatrix();
            glTranslatef(0.4, 0.0, 0.0);
            glPushMatrix(); // foot
                glRotatef (a, 0.0, 0.0, 1.0);
                    glPushMatrix();
                        glColor3f (0.35, 0.35, 0.35);
                        glScalef(0.7, 0.35, 0.35);
                        glutSolidSphere(0.2, 50, 50);
                    glPopMatrix();
            glPopMatrix();
        glPopMatrix();
    glPopMatrix();

    // body
    glPushMatrix(); 
        glRotatef (b, 0.0, 0.0, -1.0);
        glTranslatef(-0.5, 0.0, 0.0);
        glPushMatrix();
            glColor3f (0.5, 0.5, 0.5);
            glScalef(3.0, 1.2, 1.5);
            glutSolidSphere(0.2, 50, 50);
        glPopMatrix();
    glPopMatrix();

        
    glPushMatrix(); 
        glRotatef (b, 0.0, 0.0, -1.0);
        glTranslatef(-0.4, 0.0, 0.0);
        glPushMatrix(); // head
            glRotatef (b*0.5, 0.0, 0.0, -1.0);
            glTranslatef(-0.75, 0.0, 0.0);
                glPushMatrix();
                    glColor3f (headR, headG, headB);
                    glScalef(1.0, 1.0, 1.0);
                    glutSolidSphere(0.25, 50, 50);
                glPopMatrix();
        glPopMatrix();
    glPopMatrix();

        
    
    

    glPushMatrix(); // upper left leg
        glRotatef (a, 0.0, 0.0, 1.0);
        glTranslatef(0.4, 0.0, -w+0.1);
                    glPushMatrix();
                        glColor3f (0.7, 0.7, 0.7);
                        glScalef(2.0, 0.75, 0.75);
                        glutSolidSphere(0.2, 50, 50);
                    glPopMatrix();
        glPushMatrix(); // lower leg
            glTranslatef(0.4, 0.0, 0.0);
            glRotatef (-a*2.0, 0.0, 0.0, 1.0);
            glTranslatef(0.4, 0.0, 0.0);
                    glPushMatrix();
                        glColor3f (0.7, 0.7, 0.7);
                        glScalef(2.0, 0.65, 0.65);
                        glutSolidSphere(0.2, 50, 50);
                    glPopMatrix();
            glTranslatef(0.4, 0.0, 0.0);
            glPushMatrix(); // foot
                glRotatef (a, 0.0, 0.0, 1.0);
                    glPushMatrix();
                        glColor3f (0.7, 0.7, 0.7);
                        glScalef(0.7, 0.35, 0.35);
                        glutSolidSphere(0.2, 50, 50);
                    glPopMatrix();
            glPopMatrix();
        glPopMatrix();
    glPopMatrix();


    glPushMatrix(); 
        glRotatef (b, 0.0, 0.0, -1.0);
        glTranslatef(-0.5, 0.0, 0.0);
        glPushMatrix(); // upper left arm
                glTranslatef(0.0, 0.0, -w);
                glRotatef (180, 0.0, 0.0, -1.0);
                        glPushMatrix();
                            glColor3f (0.8, 0.8, 0.8);
                            glScalef(1.8, 0.5, 0.5);
                            glutSolidSphere(0.2, 50, 50);
                        glPopMatrix();
                glTranslatef(-0.35, 0.0, 0.0);
                glPushMatrix(); // lower left arm
                    glRotatef (c, 0.0, 0.0, 1.0);
                    glRotatef (d, 0.0, 1.0, 0.0);
                    glTranslatef(-0.4, 0.0, 0.0);
                        glPushMatrix();
                            glColor3f (0.8, 0.8, 0.8);
                            glScalef(1.9, 0.4, 0.4);
                            glutSolidSphere(0.2, 50, 50);
                        glPopMatrix();
                    glTranslatef(-0.35, 0.0, 0.0);
                    // hand
                    glPushMatrix();
                        glColor3f (0.8, 0.8, 0.8);
                        glScalef(0.4, 0.4, 0.4);
                        glutSolidSphere(0.2, 50, 50);
                    glPopMatrix();
                glPopMatrix();
            glPopMatrix();
        glPopMatrix();
    glPopMatrix();

   glutSwapBuffers();
}

void reshape (int w, int h)
{
   glViewport (0, 0, (GLsizei) w, (GLsizei) h); 
   glMatrixMode (GL_PROJECTION);
   glLoadIdentity ();

   gluPerspective(60.0, (GLfloat) w/(GLfloat) h, 3.0, 10.0);
   gluLookAt(0.0, -3.0, 3.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0);


   
}

void keyboard (unsigned char key, int x, int y)
{
   switch (key) {
    case 'e':
        if(a<55) a = a+10;
        glutPostRedisplay();
        break;
    case 'E':
        if(a>0) a = a-10;
        glutPostRedisplay();
        break;
    case 'r':
        if(a>40 && c>120 && d>10)
        {
            if(b<60) b = b+10;
            if(b==50) {
                if(headR<1) {
                    headR = headR+0.1;
                    headG = headG-0.1;
                    headB = headB-0.1;
                    }
                }
            if(headR>0.9) {
                a = 0; b = 0; c = 0; d = 0;
            }
        }
        else {
            if(headR > 0.44) {headR = headR - 0.05;}
            else headR = 0.4;
            if(headG < 0.44) {headG = headG + 0.05;}
            else headG = 0.4;
            if(headB < 0.44) {headB = headB + 0.05;}
            else headB = 0.4;
        }
        glutPostRedisplay();
        break;
    case 't':
        if(b>0) b = b-10;
        glutPostRedisplay();
        break;
    case 'q':
        if(c<135) c = c+20;
        glutPostRedisplay();
        break;
    case 'Q':
        if(c>0) c = c-20;
        glutPostRedisplay();
        break;
    case 'w':
        if(d<20) d = d+10;
        glutPostRedisplay();
        break;
    case 'W':
        if(d>0) d = d-10;
        glutPostRedisplay();
        break;
    case 27:
        exit(0);
        break;
    default:
        break;
   }
}

int main(int argc, char** argv)
{
   glutInit(&argc, argv);
   glutInitDisplayMode (GLUT_DOUBLE | GLUT_RGB);
   glutInitWindowSize (500, 500); 
   glutInitWindowPosition (400, 200);
   glutCreateWindow (argv[0]);
   init ();
   glutDisplayFunc(display); 
   glutReshapeFunc(reshape);
   glutKeyboardFunc(keyboard);
   glutMainLoop();
   return 0;
}
