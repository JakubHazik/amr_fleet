#include <amr_gui/control/Joypad.h>

#include <QPainter>
#include <QParallelAnimationGroup>
#include <QPropertyAnimation>
#include <QMouseEvent>
#include <QDebug>

namespace amr_gui {
template<typename T>
T constrain(T Value, T Min, T Max) {
    return (Value < Min) ? Min : (Value > Max) ? Max : Value;
}


JoyPad::JoyPad(QWidget *parent) : QWidget(parent),
                                 m_x(0), m_y(0),
                                 m_returnAnimation(new QParallelAnimationGroup(this)),
                                 m_xAnimation(new QPropertyAnimation(this, "x")),
                                 m_yAnimation(new QPropertyAnimation(this, "y")),
                                 m_alignment(Qt::AlignTop | Qt::AlignLeft) {
    m_xAnimation->setEndValue(0.f);
    m_xAnimation->setDuration(400);
    m_xAnimation->setEasingCurve(QEasingCurve::OutSine);

    m_yAnimation->setEndValue(0.f);
    m_yAnimation->setDuration(400);
    m_yAnimation->setEasingCurve(QEasingCurve::OutSine);

    m_returnAnimation->addAnimation(m_xAnimation);
    m_returnAnimation->addAnimation(m_yAnimation);
}

/**
* @brief JoyPad::x
* @return
*/
float JoyPad::x() const {
    return m_x;
}

/**
* @brief JoyPad::y
* @return
*/
float JoyPad::y() const {
    return m_y;
}

/**
* @brief JoyPad::setX
* @param value of x axis from -1 to 1
*/
void JoyPad::setX(float value) {
    m_x = constrain(value, -1.f, 1.f);

    qreal radius = (m_bounds.width() - m_knopBounds.width()) / 2;
    m_knopBounds.moveCenter(QPointF(m_bounds.center().x() + m_x * radius, m_knopBounds.center().y()));

    update();
    emit xChanged(m_x);
}

/**
* @brief JoyPad::setY
* @param value of y axis from -1 to 1
*/
void JoyPad::setY(float value) {
    m_y = constrain(value, -1.f, 1.f);

    qreal radius = (m_bounds.width() - m_knopBounds.width()) / 2;
    m_knopBounds.moveCenter(QPointF(m_knopBounds.center().x(), m_bounds.center().y() - m_y * radius));

    update();
    emit yChanged(m_y);
}

void JoyPad::removeXAnimation() {
    // return if the animation is already removed
    if (m_xAnimation->parent() != m_returnAnimation) return;

    m_returnAnimation->removeAnimation(m_xAnimation);

    // take ownership of the animation (parent is 0 after removeAnimation())
    m_xAnimation->setParent(this);
}

void JoyPad::addXAnimation() {
    // abort if the animation is already added
    if (m_xAnimation->parent() == m_returnAnimation) return;

    m_returnAnimation->addAnimation(m_xAnimation);
}

void JoyPad::removeYAnimation() {
    if (m_yAnimation->parent() != m_returnAnimation) return;

    m_returnAnimation->removeAnimation(m_yAnimation);
    m_yAnimation->setParent(this);
}

void JoyPad::addYAnimation() {
    if (m_yAnimation->parent() == m_returnAnimation) return;

    m_returnAnimation->addAnimation(m_yAnimation);
}

void JoyPad::setAlignment(Qt::Alignment f) {
    m_alignment = f;
}

/**
* @brief JoyPad::resizeEvent
* @param event
*
* calculates a square bounding rect for the background and the knob
*/
void JoyPad::resizeEvent(QResizeEvent *event) {
    Q_UNUSED(event)

    float a = qMin(width(), height());

    QPointF topleft;

    if (m_alignment.testFlag(Qt::AlignTop)) {
        topleft.setY(0);
    } else if (m_alignment.testFlag(Qt::AlignVCenter)) {
        topleft.setY(((height() - a) / 2));
    } else if (m_alignment.testFlag(Qt::AlignBottom)) {
        topleft.setY(height() - a);
    }

    if (m_alignment.testFlag(Qt::AlignLeft)) {
        topleft.setX(0);
    } else if (m_alignment.testFlag(Qt::AlignHCenter)) {
        topleft.setX((width() - a) / 2);
    } else if (m_alignment.testFlag(Qt::AlignRight)) {
        topleft.setX(width() - a);
    }

    m_bounds = QRectF(topleft, QSize(a, a));
    qDebug() << m_bounds;

    m_knopBounds.setWidth(a * 0.3);
    m_knopBounds.setHeight(a * 0.3);

    // adjust knob position
    qreal radius = (m_bounds.width() - m_knopBounds.width()) / 2;
    m_knopBounds.moveCenter(QPointF(m_bounds.center().x() + m_x * radius, m_bounds.center().y() - m_y * radius));
}

/**
* @brief JoyPad::paintEvent
* @param event
*/
void JoyPad::paintEvent(QPaintEvent *event) {
    Q_UNUSED(event)

    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);
    painter.setRenderHint(QPainter::HighQualityAntialiasing);

    // draw background
    QRadialGradient gradient(m_bounds.center(), m_bounds.width() / 2, m_bounds.center());
    gradient.setFocalRadius(m_bounds.width() * 0.3);
    gradient.setCenterRadius(m_bounds.width() * 0.7);
    gradient.setColorAt(0, Qt::white);
    gradient.setColorAt(1, Qt::lightGray);

    painter.setPen(QPen(QBrush(Qt::gray), m_bounds.width() * 0.005));
    painter.setBrush(QBrush(gradient));
    painter.drawEllipse(m_bounds);

    // draw crosshair
    painter.setPen(QPen(QBrush(Qt::gray), m_bounds.width() * 0.005));
    painter.drawLine(QPointF(m_bounds.left(), m_bounds.center().y()),
                     QPointF(m_bounds.center().x() - m_bounds.width() * 0.35, m_bounds.center().y()));
    painter.drawLine(QPointF(m_bounds.center().x() + m_bounds.width() * 0.35, m_bounds.center().y()),
                     QPointF(m_bounds.right(), m_bounds.center().y()));
    painter.drawLine(QPointF(m_bounds.center().x(), m_bounds.top()),
                     QPointF(m_bounds.center().x(), m_bounds.center().y() - m_bounds.width() * 0.35));
    painter.drawLine(QPointF(m_bounds.center().x(), m_bounds.center().y() + m_bounds.width() * 0.35),
                     QPointF(m_bounds.center().x(), m_bounds.bottom()));

    // draw knob
    if (!this->isEnabled()) return;

    gradient = QRadialGradient(m_knopBounds.center(), m_knopBounds.width() / 2, m_knopBounds.center());
    gradient.setColorAt(0, Qt::gray);
    gradient.setColorAt(1, Qt::darkGray);
    gradient.setFocalRadius(m_knopBounds.width() * 0.2);
    gradient.setCenterRadius(m_knopBounds.width() * 0.5);

    painter.setPen(QPen(QBrush(Qt::darkGray), m_bounds.width() * 0.005));
    painter.setBrush(QBrush(gradient));
    painter.drawEllipse(m_knopBounds);
}

/**
* @brief JoyPad::mousePressEvent
* @param event
*/
void JoyPad::mousePressEvent(QMouseEvent *event) {
    if (m_knopBounds.contains(event->pos())) {
        m_returnAnimation->stop();
        m_lastPos = event->pos();
        knopPressed = true;
    }
}

/**
* @brief JoyPad::mouseReleaseEvent
* @param event
*/
void JoyPad::mouseReleaseEvent(QMouseEvent *event) {
    Q_UNUSED(event)

    knopPressed = false;
    m_returnAnimation->start();
}

/**
* @brief JoyPad::mouseMoveEvent
* @param event
*/
void JoyPad::mouseMoveEvent(QMouseEvent *event) {
    if (!knopPressed) return;

    // moved distance
    QPointF dPos = event->pos() - m_lastPos;

    // change the distance sligthly to guarantee overlaping knop and pointer
    dPos += 0.05 * (event->pos() - m_knopBounds.center());

    QPointF fromCenterToKnop = m_knopBounds.center() + dPos - m_bounds.center();

    qreal radius = (m_bounds.width() - m_knopBounds.width()) / 2;

    fromCenterToKnop.setX(constrain(fromCenterToKnop.x(), -radius, radius));
    fromCenterToKnop.setY(constrain(fromCenterToKnop.y(), -radius, radius));

    m_knopBounds.moveCenter(fromCenterToKnop + m_bounds.center());
    m_lastPos = event->pos();

    update();

    if (radius == 0) return;
    float x = (m_knopBounds.center().x() - m_bounds.center().x()) / radius;
    float y = (-m_knopBounds.center().y() + m_bounds.center().y()) / radius;

    if (m_x != x) {
        m_x = x;
        emit xChanged(m_x);
    }

    if (m_y != y) {
        m_y = y;
        emit yChanged(m_y);
    }
}

}

