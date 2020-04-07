#pragma once

#include <QWidget>

class QPropertyAnimation;
class QParallelAnimationGroup;

namespace amr_gui {

/**
 *  How to use:

    In parent widget write:
    connect(ui->widget, &JoyPad::xChanged, this, [this](float x){
        qDebug() << "x: " << x << " y: " << ui->widget->y();
    });

 */
class JoyPad : public QWidget
{
    Q_OBJECT
    Q_PROPERTY(double x READ x WRITE setX NOTIFY xChanged)
    Q_PROPERTY(double y READ y WRITE setY NOTIFY yChanged)
public:
    explicit JoyPad(QWidget *parent = Q_NULLPTR);

    double x() const;
    double y() const;

Q_SIGNALS:
    void xChanged(double value);
    void yChanged(double value);

public Q_SLOTS:
    void setX(double value);
    void setY(double value);

    // Add or remove the knob return animations in x or y- direction.
    void removeXAnimation();
    void addXAnimation();

    void removeYAnimation();
    void addYAnimation();

    /*  Set the alignment of the quadratic content if the widgets geometry isn quadratic.
     *  Flags can be combined eg. setAlignment(Qt::AlignLeft | Qt::AlignBottom);
    */
    void setAlignment(Qt::Alignment f);

private:
    void resizeEvent(QResizeEvent *event) override;
    virtual void paintEvent(QPaintEvent *event) override;
    void mousePressEvent(QMouseEvent *event) override;
    void mouseReleaseEvent(QMouseEvent *event) override;
    void mouseMoveEvent(QMouseEvent *event) override;

    double m_x;
    double m_y;

    QParallelAnimationGroup *m_returnAnimation;
    QPropertyAnimation *m_xAnimation;
    QPropertyAnimation *m_yAnimation;

    QRectF m_bounds;
    QRectF m_knopBounds;

    QPoint m_lastPos;
    bool knopPressed;

    Qt::Alignment m_alignment;
};
}