#include <QHHeadersGLUT.h> 

int main(int argc, char* argv[])
{
    QHGLUT* DisplayObject = new QHGLUT(argc, argv);
    DeviceSpace* Omni = new DeviceSpace;
    DisplayObject->tell(Omni);

    Sphere* gMySphere = new Sphere(25.0, 30.0);
    gMySphere->setStiffness(1.0);
    gMySphere->setFriction(0.4, 0.5);

    gMySphere->setTranslation(0.0, 0.0, 0.0);
    gMySphere->setShapeColor(0.9, 0.2, 0.2);
    DisplayObject->tell(gMySphere);

    Cursor* OmniCursor = new Cursor;
    DisplayObject->tell(OmniCursor);

    char* descriptionText = (char*)"A solid sphere, compiled cleanly!";
    Text* description = new Text(20.0, descriptionText, 0.1, 0.9);
    description->setShapeColor(1.0, 1.0, 1.0);
    DisplayObject->tell(description);
    
    qhStart();

    return 0;
}