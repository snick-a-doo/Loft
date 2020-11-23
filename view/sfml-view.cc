#include <body.hh>
#include <units.hh>
#include <universe.hh>
#include <world.hh>

#include <GL/glu.h>
#include <GL/glut.h>
#include <SFML/Graphics.hpp>
#include <SFML/OpenGL.hpp>
#include <SFML/Window.hpp>

#include <cassert>
#include <iomanip>
#include <sstream>
#include <vector>

std::string format_time(int seconds)
{
    int days = seconds/(24*3600);
    seconds -= days*24*3600;
    int hours = seconds/3600;
    seconds -= hours*3600;
    int minutes = seconds/60;
    seconds -= minutes*60;
    std::ostringstream os;
    os << days << "d "
       << std::setw(2) << std::setfill('0')
       << hours << ':'
       << std::setw(2) << std::setfill('0')
       << minutes << ':'
       << std::setw(2) << std::setfill('0')
       << seconds;
    return os.str();
}

template <typename T>
void texxt(double x, double y, const std::string& label, const T& value,
          const std::string& units = "", int precision = 0)
{
    std::ostringstream os;
    os.setf(std::ios::fixed);
    os << std::setprecision(precision) << label << ' ' << value << ' ' << units;
    glColor3f(1.0, 1.0, 1.0);
    glRasterPos2d(x, y);
    for (auto c : os.str())
        glutBitmapCharacter(GLUT_BITMAP_8_BY_13, c);
}

struct View
{
    double x;
    double width;
    double y;
    double height;
    V3 eye;
    V3 up;
};

int main(int argc, char** argv)
{
    using namespace consts;

    auto orientation = rot(M1, units::deg(23.44)*Vy);
    auto earth = std::make_shared<World>(m_earth, r_earth, V0, V0, orientation,
                                         units::day(1.0));
    // auto moon = std::make_shared<World>(m_moon, r_moon, 4.054e8*Vx, 0.97e3*Vy, M1,
                                     // units::day(27.32));
    auto ksc_lat = units::dms(28, 31, 27);
    auto ksc_lon = units::dms(-80, 39, 03);
    auto greenwich_lat = units::dms(51, 28, 40);
    auto body = std::make_shared<Body>(100.0, M1, earth->locate(ksc_lat, ksc_lon, 0),
                                       V0, M1, V0);
    Universe all;
    all.add(earth);
    all.add(body);
    // all.add(moon);

    // earth->capture(body);

    std::array<View, 4> views {View(0.0, 0.5, 0.0, 0.5, Vz, Vy),
                               View(0.5, 0.5, 0.0, 0.5, Vx, Vz),
                               View(0.0, 0.5, 0.5, 0.5, Vy, Vz),
                               View(0.5, 0.5, 0.5, 0.5, -Vx, Vz)};

    // create the window
    double width = 800;
    double height = 600;
    sf::RenderWindow window(sf::VideoMode(width, height), "Loft",
                      sf::Style::Default, sf::ContextSettings(24));
    window.setVerticalSyncEnabled(true);
    window.setActive(true);

    glutInit(&argc, argv);
    auto* earth_quad = gluNewQuadric();
    gluQuadricDrawStyle(earth_quad, GLU_FILL);
    gluQuadricNormals(earth_quad, GLU_SMOOTH);
    gluQuadricTexture(earth_quad, GL_TRUE);
    auto* body_quad = gluNewQuadric();
    gluQuadricDrawStyle(body_quad, GLU_FILL);
    gluQuadricNormals(body_quad, GLU_SMOOTH);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_TEXTURE_2D);
    glClearColor(0.2, 0.2, 0.2, 0.0);
    glMatrixMode(GL_PROJECTION);

    std::vector<V3> ground;

    sf::Texture earth_tex;
    if (!earth_tex.loadFromFile("earth-texture.png"))
        exit(2);
    sf::Clock clock;
    bool running = true;
    bool once = true;
    while (running)
    {
        sf::Event event;
        while (window.pollEvent(event))
        {
            switch (event.type)
            {
            case sf::Event::Closed:
                running = false;
                break;
            case sf::Event::Resized:
            {
                // adjust the viewport when the window is resized
                width = event.size.width;
                height = event.size.height;
                break;
            }
            case sf::Event::KeyPressed:
                if (event.key.code == sf::Keyboard::Escape)
                    running = false;
                break;
            default:
                break;
            }
        }

        double scale = 4*r_earth;
        auto elapsed = clock.restart().asSeconds();
        all.step(1e3*elapsed);
        auto r = body->transform_out(V0);
        auto [axis, angle] = axis_angle(earth->orientation());
        if (all.time() > 2e2 && once)
        {
            std::cout << "go" << std::endl;
            earth->release(body);
            body->impulse(10.2e5*Vz);
            once = false;
        }

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        for (const auto& v : views)
        {
            auto w = width*v.width;
            auto h = height*v.height;
            glViewport(width*v.x, height*v.y, w, h);
            double dim = std::min(w, h);
            glLoadIdentity();
            glOrtho(-scale*w/dim, scale*w/dim,
                    -scale*h/dim, scale*h/dim,
                    -2*scale, 2*scale);
            gluLookAt(v.eye.x, v.eye.y, v.eye.z, 0, 0, 0, v.up.x, v.up.y, v.up.z);

            glPushMatrix();
            glColor3d(1.0, 1.0, 1.0);
            sf::Texture::bind(&earth_tex);
            glRotated((180.0/pi)*angle, axis.x, axis.y, axis.z);
            gluSphere(earth_quad, r_earth, 128, 128);
            ground.push_back(1.01*earth->radius()*unit(earth->transform_in(r - earth->r())));
            glColor3f(1.0, 0.0, 1.0);
            glBegin(GL_LINE_STRIP);
            for (auto& v : ground)
                glVertex3f(v.x, v.y, v.z);
            glEnd();
            glPopMatrix();

            glPushMatrix();
            glColor3d(0.8, 0.8, 0.2);
            // glTranslatef(moon->r().x, moon->r().y, moon->r().z);
            // gluSphere(quad, 0.6*r_earth, 64, 64);
            glPopMatrix();

            glPushMatrix();
            glColor3d(0.2, 0.8, 0.2);
            glTranslated(r.x, r.y, r.z);
            gluSphere(body_quad, 0.02*scale, 32, 32);
            glPopMatrix();

            glPushMatrix();
            glLoadIdentity();
            sf::Texture::bind(nullptr);
            glOrtho(0, width, 0, height, -1, 1);
            // texxt(0, 0, "Time:", format_time(all.time()));
            texxt(0, height-30, "", v.eye);
            texxt(0, 0, "", r, "", 1);
            // texxt(100, 0, "Y:", v.y, "", 1);
            glPopMatrix();
        }
        glFlush();

        window.display();
    }
    return 0;
}
