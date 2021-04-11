#ifndef MY_THREAD_H
#define MY_THREAD_H

#include <QObject>

class my_thread : public QObject
{
    Q_OBJECT
public:
    explicit my_thread(QObject *parent = nullptr);

signals:

public slots:
    void on_server();
};

#endif // MY_THREAD_H
