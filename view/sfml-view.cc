#include <body.hh>
#include <rocket.hh>
#include <units.hh>
#include <universe.hh>
#include <world.hh>

#include <GL/glu.h>
#include <GL/glut.h>
#include <SFML/Graphics.hpp>
#include <SFML/OpenGL.hpp>
#include <SFML/Window.hpp>

#include <cassert>
#include <fstream>
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
void draw_text(double x, double y, const std::string& label, const T& value,
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
    double y;
    double width;
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
    auto ksc_lat = units::dms(28, 31, 27);
    auto ksc_lon = units::dms(-80, 39, 03);
    auto greenwich_lat = units::dms(51, 28, 40);
    auto [r, m] = earth->locate(ksc_lat, ksc_lon, 10);
    auto body = std::make_shared<Rocket>(10, 50, 0.5, 10,
                                         1.2, 8.0e4, 0.01, r, m);
    body->throttle(1.0);
    Universe all;
    all.add(earth);
    all.add(body);

    earth->capture(body);

    std::array<View, 4> views {View(0.0, 0.5, 0.5, 0.5, Vz, Vy),
                               View(0.5, 0.5, 0.5, 0.5, V0, V0),
                               View(0.0, 0.0, 0.5, 0.5, Vx, Vz),
                               View(0.5, 0.0, 0.5, 0.5, Vy, Vz)};

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

    // Position history for drawing tracks.
    std::vector<V3> ground;
    std::vector<V3> map;
    std::vector<V3> air;

    sf::Texture earth_tex;
    if (!earth_tex.loadFromFile("earth-texture.png"))
        exit(2);
    sf::Clock clock;
    bool running = true;
    int stage = 0;
    int n = 0;
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

        double scale = 1.5*r_earth;
        auto elapsed = clock.restart().asSeconds();
        all.step(1e2*elapsed);
        auto r = body->transform_out(V0);
        auto z_hat = body->rotate_out(Vz);
        auto [axis, angle] = axis_angle(earth->orientation());
        auto [body_axis, body_angle] = axis_angle(body->orientation());
        if (all.time() > 1 && stage == 0)
        {
            // std::cout << "go" << std::endl;
            earth->release(body);
            ++stage;
        }
        else if (all.time() > 110 && stage == 1)
        {
            // std::cout << "turn" << std::endl;
            body->orient_thrust(-2e-5*Vy);
            ++stage;
        }
        else if (all.time() > 110 && stage == 2)
        {
            // std::cout << "unturn" << std::endl;
            // body->orient_thrust(-1e-4*Vy);
            ++stage;
        }
        else if (all.time() > 142 && stage == 3)
        {
            // std::cout << "straight" << std::endl;
            body->orient_thrust(V0);
            ++stage;
        }

        // Add tracking points at intervals to avoid filling the vectors.
        if (n++ == 10)
        {
            n = 0;
            air.push_back(r);
            ground.push_back(1.01*earth->radius()*unit(earth->transform_in(r - earth->r())));
            auto [lat, lon, alt] = earth->location(r);
            map.push_back(V3(lon, lat, alt)); // longitude 1st because it's x-like.
        }

        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        for (const auto& v : views)
        {
            auto w = width*v.width;
            auto h = height*v.height;
            glViewport(width*v.x, height*v.y, w, h);
            double dim = std::min(w, h);
            glLoadIdentity();
            if (v.eye == V0)
            {
                glOrtho(-pi, pi, -pi/2, pi/2, -1, 1);
                glColor3f(1.0, 1.0, 1.0);
                sf::Texture::bind(&earth_tex);
                glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
                glBegin(GL_QUADS);
                glTexCoord2d(-0.5, 0);
                glVertex3d(-pi, -pi/2, 0);
                glTexCoord2d(0.5, 0);
                glVertex3d(pi, -pi/2, 0);
                glTexCoord2d(0.5, 1);
                glVertex3d(pi, pi/2, 0);
                glTexCoord2d(-0.5, 1);
                glVertex3d(-pi, pi/2, 0);
                glEnd();
                sf::Texture::bind(nullptr);
                glColor3f(1.0, 0.0, 1.0);
                glBegin(GL_LINE_STRIP);
                double last_x = -2*pi;
                for (auto& v : map)
                {
                    // Break the strip when wrapping from pi to -pi or -pi to pi.
                    if (std::abs(v.x - last_x) > 3)
                    {
                        glEnd();
                        glBegin(GL_LINE_STRIP);
                    }
                    glVertex3d(v.x, v.y, 0.5);
                    last_x = v.x;
                }
                glEnd();
                continue;
            }
            glOrtho(-scale*w/dim, scale*w/dim,
                    -scale*h/dim, scale*h/dim,
                    -2*scale, 2*scale);
            gluLookAt(v.eye.x, v.eye.y, v.eye.z, 0, 0, 0, v.up.x, v.up.y, v.up.z);

            glColor3f(1.0, 1.0, 0.0);
            glBegin(GL_LINE_STRIP);
            for (auto& v : air)
                glVertex3f(v.x, v.y, v.z);
            glEnd();

            glPushMatrix();
            glColor3d(1.0, 1.0, 1.0);
            sf::Texture::bind(&earth_tex);
            glRotated((180.0/pi)*angle, axis.x, axis.y, axis.z);
            gluSphere(earth_quad, earth->radius(), 128, 128);
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
            glRotated((180.0/pi)*body_angle, body_axis.x, body_axis.y, body_axis.z);
            gluCylinder(body_quad, r_earth/40, r_earth/40, r_earth/10, 32, 1);
            glPopMatrix();

            glPushMatrix();
            glLoadIdentity();
            sf::Texture::bind(nullptr);
            glOrtho(0, width, 0, height, -1, 1);
            draw_text(0, height-30, "", v.eye);
            draw_text(0, 0, "Vel:", 1e-3*mag(body->v_cm()), "km/s", 3);
            draw_text(0, 20, "Alt:", 1e-3*(mag(body->r()) - earth->radius()), "km", 0);
            draw_text(0, 40, "Fuel:", body->fuel_volume(), "m^3", 3);
            glPopMatrix();
        }
        glFlush();
        window.display();
        sf::sleep(sf::milliseconds(20));
    }
    return 0;
}
