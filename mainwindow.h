#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QMessageBox>
#include <QtSerialPort/QSerialPort>
#include <QtNetwork/QUdpSocket>
#include <QLabel>
#include <QInputDialog>
#include <QTimer>
#include "settingsdialog.h"

#include <QNetworkDatagram>
#include <QFile>
#include <QTextStream>
#include <QTime>
#include <QDate>
#include <QDir>
#include <QtCharts>
#include <QSplineSeries>
#include <QChartView>
#include <QGridLayout>
#include <QList>
#include <QDialog>
#include <QElapsedTimer>
#include <QPalette>
#include <QColor>

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void on_pushButton_ALIVE_clicked();

    void openSerialPorts();

    void closeSerialPorts();

    void dataRecived();

    void decodeData(uint8_t *datosRx, uint8_t source);

    void sendDataSerial();

    void timeOut();

    void on_pushButton_OPENUDP_clicked();

    void sendDataUDP();

    void OnUdpRxData();

    void on_pushButtonClearDebugger_clicked();

    void on_pushButton_SETEO_DE_CONTROL_clicked();

    //Muestreo de datos ACCEL
    void createChartACCEL();
    void addPointChartACCELX(uint16_t point);
    void addPointChartACCELY(uint16_t point);
    void addPointChartACCELZ(uint16_t point);

    //Muestreo de datos GYRO
    void createChartGYRO();
    void addPointChartGYROX(uint16_t point);
    void addPointChartGYROY(uint16_t point);
    void addPointChartGYROZ(uint16_t point);

private:
    Ui::MainWindow *ui;
    QSerialPort *serial;
    SettingsDialog *settingPorts;
    QLabel *estadoSerial;
    QTimer  *timer1;

    QUdpSocket *UdpSocket1;
    QHostAddress RemoteAddress;
    quint16 RemotePort;
    QHostAddress clientAddress;
    int puertoremoto;

    typedef enum{
        START,
        HEADER_1,
        HEADER_2,
        HEADER_3,
        NBYTES,
        TOKEN,
        PAYLOAD
    }_eProtocolo;

    _eProtocolo estadoProtocolo,estadoProtocoloUdp;

    typedef enum{
        UDP=0,
        SERIE=1,
        ACK=0x0D,
        GETALIVE=0xF0,
        GETFIRMWARE=0xF1,
        UNKNOWCMD=0xFF,
        SETLEDS=0x10,
        GETSWITCHES=0x12,
        GETANALOGSENSORS=0xA0,
        GETMPU=0xA1,
        GETDATACONTROL=0xA2,
        SETDATACONTROL=0xA3,
        SETFREQ=0xA4,
        SETROBOTMODE=0xA5,
        GETPID=0xA6,
        SETWHITECOLORDETECTED=0xA7,
        SERVOMOVESTOP=0x0A,
        OTHERS
    }_eCmd;


    typedef struct{
        uint8_t timeOut;
        uint8_t cheksum;
        uint8_t payLoad[256];
        uint8_t nBytes;
        uint8_t index;
    }_sDatos ;

    _sDatos rxData, rxDataUdp;

    typedef union {
        double  d32;
        float f32;
        int i32;
        unsigned int ui32;
        unsigned short ui16[2];
        short i16[2];
        uint8_t ui8[4];
        char chr[4];
        unsigned char uchr[4];
        int8_t  i8[4];
    }_udat;

    _udat myWord;

    int contadorAlive=0;

    uint32_t ADCIR[8];

    float ACCELMPU[3];
    float GYROMPU[3];

    float KP_IZQ;
    float KD_IZQ;
    float KI_IZQ;
    int16_t VEL_IZQ;

    float KP_DER;
    float KD_DER;
    float KI_DER;
    int16_t VEL_DER;

    float sensorValue, numerador, denominador, cuenta, e_t;

    int16_t FREQ;

    int8_t robotMode;

    // Grafico del ACCEL
    QSplineSeries *accelXSpline;
    QSplineSeries *accelYSpline;
    QSplineSeries *accelZSpline;

    QChart *accelChart;
    QChartView *accelChartView;
    QGridLayout *accelLayout;

    QList<QPointF> accelXDatos;
    QList<QPointF> accelYDatos;
    QList<QPointF> accelZDatos;

    // Grafico del GYRO
    QSplineSeries *gyroXSpline;
    QSplineSeries *gyroYSpline;
    QSplineSeries *gyroZSpline;

    QChart *gyroChart;
    QChartView *gyroChartView;
    QGridLayout *gyroLayout;

    QList<QPointF> gyroXDatos;
    QList<QPointF> gyroYDatos;
    QList<QPointF> gyroZDatos;

};
#endif // MAINWINDOW_H
