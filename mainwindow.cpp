#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    serial=new QSerialPort(this);
    settingPorts=new SettingsDialog(this);
    estadoSerial = new QLabel(this);
    estadoSerial->setText("Desconectado ......");
    ui->statusbar->addWidget(estadoSerial);
    ui->actionDisconnect_Device->setEnabled(false);
    timer1=new QTimer(this);
    UdpSocket1 = new QUdpSocket(this);

    connect(ui->actionQuit,&QAction::triggered,this,&MainWindow::close);
    connect(ui->actionScanPorts, &QAction::triggered, settingPorts,&SettingsDialog::show);
    connect(ui->actionConnect_Device, &QAction::triggered,this,&MainWindow::openSerialPorts);
    connect(ui->actionDisconnect_Device, &QAction::triggered, this, &MainWindow::closeSerialPorts);
    connect(ui->pushButton_SEND,&QPushButton::clicked, this, &MainWindow::sendDataSerial);
    connect(serial,&QSerialPort::readyRead,this,&MainWindow::dataRecived);
    connect(timer1,&QTimer::timeout,this,&MainWindow::timeOut);
    connect(UdpSocket1,&QUdpSocket::readyRead,this,&MainWindow::OnUdpRxData);
    connect(ui->pushButton_UDP,&QPushButton::clicked,this,&MainWindow::sendDataUDP);

    ui->comboBox_CMD->addItem("ALIVE", 0xF0);
    ui->comboBox_CMD->addItem("FIRMWARE", 0xF1);
    ui->comboBox_CMD->addItem("SENSORES", 0xA0);
    ui->comboBox_CMD->addItem("MPU", 0xA1);
    ui->comboBox_CMD->addItem("GET DATA CONTROL", 0xA2);
    ui->comboBox_CMD->addItem("SET DATA CONTROL", 0xA3);
    //ui->comboBox_CMD->addItem("SETFREQ", 0xA4);
    ui->comboBox_CMD->addItem("SET ROBOT MODE", 0xA5);
    ui->comboBox_CMD->addItem("GET PID", 0xA6);

    ui->comboBox_MODE->addItem("IDLE", 0);
    ui->comboBox_MODE->addItem("RUN", 1);

    estadoProtocolo=START;
    rxData.timeOut=0;
    ui->pushButton_UDP->setEnabled(false);
    ui->pushButton_ALIVE->setEnabled(false);
    ui->pushButton_SEND->setEnabled(false);
    timer1->start(100);

    //Graficos
    createChartACCEL();
    createChartGYRO();
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::openSerialPorts(){
    const SettingsDialog::Settings p = settingPorts->settings();
    serial->setPortName(p.name);
    serial->setBaudRate(p.baudRate);
    serial->setDataBits(p.dataBits);
    serial->setParity(p.parity);
    serial->setStopBits(p.stopBits);
    serial->setFlowControl(p.flowControl);
    serial->open(QSerialPort::ReadWrite);
    if (serial->isOpen()){
        ui->pushButton_ALIVE->setEnabled(true);
        ui->pushButton_SEND->setEnabled(true);
        ui->actionDisconnect_Device->setEnabled(true);
        estadoSerial->setStyleSheet("QLabel { color : blue; }");
        estadoSerial->setText(tr("Connected to %1 : %2, %3, %4, %5, %6, %7")
                                  .arg(p.name)
                                  .arg(p.stringBaudRate)
                                  .arg(p.stringDataBits)
                                  .arg(p.stringParity)
                                  .arg(p.stringStopBits)
                                  .arg(p.stringFlowControl)
                                  .arg(p.fabricante)
                              );
    } else {
        QMessageBox::critical(this, tr("Error"), serial->errorString());
    }
}

void MainWindow::closeSerialPorts(){
    ui->actionDisconnect_Device->setEnabled(false);
    estadoSerial->setText("Desconectado ......");
    if (serial->isOpen())
        serial->close();
}

void MainWindow::dataRecived(){
    unsigned char *incomingBuffer;
    int count;

    count = serial->bytesAvailable();

    if(count<=0)
        return;

    incomingBuffer = new unsigned char[count];

    serial->read((char *)incomingBuffer,count);

    QString str="";

    for(int i=0; i<=count; i++){
        if(isalnum(incomingBuffer[i]))
            str = str + QString("%1").arg((char)incomingBuffer[i]);
        else
            str = str +"{" + QString("%1").arg(incomingBuffer[i],2,16,QChar('0')) + "}";
    }
    ui->textEdit_RAW->append("MBED-->SERIAL-->PC (" + str + ")");

    //Cada vez que se recibe un dato reinicio el timeOut
    rxData.timeOut=6;

    for(int i=0;i<count; i++){
        switch (estadoProtocolo) {
        case START:
            if (incomingBuffer[i]=='U'){
                estadoProtocolo=HEADER_1;
            }
            break;
        case HEADER_1:
            if (incomingBuffer[i]=='N')
                estadoProtocolo=HEADER_2;
            else{
                i--;
                estadoProtocolo=START;
            }
            break;
        case HEADER_2:
            if (incomingBuffer[i]=='E')
                estadoProtocolo=HEADER_3;
            else{
                i--;
                estadoProtocolo=START;
            }
            break;
        case HEADER_3:
            if (incomingBuffer[i]=='R')
                estadoProtocolo=NBYTES;
            else{
                i--;
                estadoProtocolo=START;
            }
            break;
        case NBYTES:
            rxData.nBytes=incomingBuffer[i];
            estadoProtocolo=TOKEN;
            break;
        case TOKEN:
            if (incomingBuffer[i]==':'){
                estadoProtocolo=PAYLOAD;
                rxData.cheksum='U'^'N'^'E'^'R'^ rxData.nBytes^':';
                rxData.payLoad[0]=rxData.nBytes;
                rxData.index=1;
            }
            else{
                i--;
                estadoProtocolo=START;
            }
            break;
        case PAYLOAD:
            if (rxData.nBytes>1){
                rxData.payLoad[rxData.index++]=incomingBuffer[i];
                rxData.cheksum^=incomingBuffer[i];
            }
            rxData.nBytes--;
            if(rxData.nBytes==0){
                estadoProtocolo=START;
                if(rxData.cheksum==incomingBuffer[i]){
                    decodeData(&rxData.payLoad[0], SERIE);
                }else{
                    ui->textEdit_RAW->append("Chk Calculado ** " +QString().number(rxData.cheksum,16) + " **" );
                    ui->textEdit_RAW->append("Chk recibido ** " +QString().number(incomingBuffer[i],16) + " **" );

                }
            }
            break;
        default:
            estadoProtocolo=START;
            break;
        }
    }
    delete [] incomingBuffer;

}

void MainWindow::decodeData(uint8_t *datosRx, uint8_t source){
    int32_t length = sizeof(*datosRx)/sizeof(datosRx[0]);
    QString str, strOut;
    _udat w;
    for(int i = 1; i<length; i++){
        if(isalnum(datosRx[i]))
            str = str + QString("%1").arg(char(datosRx[i]));
        else
            str = str +QString("%1").arg(datosRx[i],2,16,QChar('0'));
    }
    ui->textEdit_RAW->append("*(MBED-S->PC)->decodeData (" + str + ")");

    switch (datosRx[1]) {
    case GETANALOGSENSORS://     ANALOGSENSORS=0xA0,
        w.ui8[0] = datosRx[2];
        w.ui8[1] = datosRx[3];

        ADCIR[0] = w.ui16[0];

        w.ui8[0] = datosRx[4];
        w.ui8[1] = datosRx[5];

        ADCIR[1] = w.ui16[0];

        w.ui8[0] = datosRx[6];
        w.ui8[1] = datosRx[7];

        ADCIR[2] = w.ui16[0];

        w.ui8[0] = datosRx[8];;
        w.ui8[1] = datosRx[9];

        ADCIR[3] = w.ui16[0];

        w.ui8[0] = datosRx[10];
        w.ui8[1] = datosRx[11];

        ADCIR[4] = w.ui16[0];

        w.ui8[0] = datosRx[12];
        w.ui8[1] = datosRx[13];;

        ADCIR[5] = w.ui16[0];

        w.ui8[0] = datosRx[14];
        w.ui8[1] = datosRx[15];

        ADCIR[6] = w.ui16[0];

        w.ui8[0] = datosRx[16];
        w.ui8[1] = datosRx[17];

        ADCIR[7] = w.ui16[0];

        ui->textEdit_PROCCES->append(QString::number(ADCIR[0]) + " | "
                                     + QString::number(ADCIR[1]) + " | "
                                     + QString::number(ADCIR[2]) + " | "
                                     + QString::number(ADCIR[3]) + " | "
                                     + QString::number(ADCIR[4]) + " | "
                                     + QString::number(ADCIR[5]) + " | "
                                     + QString::number(ADCIR[6]) + " | "
                                     + QString::number(ADCIR[7]));

        break;
    case GETMPU://     MPU=0xA1,
        //ACCELMPU-------------------------------------------------------------
        w.ui8[0] = datosRx[2];
        w.ui8[1] = datosRx[3];
        w.ui8[2] = datosRx[4];
        w.ui8[3] = datosRx[5];
        ACCELMPU[0] = w.f32;
        ui->label_Ax->setText(QString::number(ACCELMPU[0], 'f', 2));
        addPointChartACCELX(ACCELMPU[0]);

        w.ui8[0] = datosRx[6];
        w.ui8[1] = datosRx[7];
        w.ui8[2] = datosRx[8];
        w.ui8[3] = datosRx[9];
        ACCELMPU[1] = w.f32;
        ui->label_Ay->setText(QString::number(ACCELMPU[1], 'f', 2));
        addPointChartACCELY(ACCELMPU[1]);

        w.ui8[0] = datosRx[10];
        w.ui8[1] = datosRx[11];
        w.ui8[2] = datosRx[12];
        w.ui8[3] = datosRx[13];
        ACCELMPU[2] = w.f32;
        ui->label_Az->setText(QString::number(ACCELMPU[2], 'f', 2));
        addPointChartACCELZ(ACCELMPU[2]);

        ui->textEdit_PROCCES->append("Ax: " + QString::number(ACCELMPU[0], 'f', 2)
                                     + "  Ay: " + QString::number(ACCELMPU[1], 'f', 2)
                                     + "  Az: " + QString::number(ACCELMPU[2], 'f', 2));

        //GYROMPU-------------------------------------------------------------
        w.ui8[0] = datosRx[14];
        w.ui8[1] = datosRx[15];
        w.ui8[2] = datosRx[16];
        w.ui8[3] = datosRx[17];
        GYROMPU[0] = w.f32;
        ui->label_Gx->setText(QString::number(GYROMPU[0], 'f', 2));
        addPointChartGYROX(GYROMPU[0]);

        w.ui8[0] = datosRx[18];
        w.ui8[1] = datosRx[19];
        w.ui8[2] = datosRx[20];
        w.ui8[3] = datosRx[21];
        GYROMPU[1] = w.f32;
        ui->label_Gy->setText(QString::number(GYROMPU[1], 'f', 2));
        addPointChartGYROY(GYROMPU[1]);

        w.ui8[0] = datosRx[22];
        w.ui8[1] = datosRx[23];
        w.ui8[2] = datosRx[24];
        w.ui8[3] = datosRx[25];
        GYROMPU[2] = w.f32;
        ui->label_Gz->setText(QString::number(GYROMPU[2], 'f', 2));
        addPointChartGYROZ(GYROMPU[2]);

        ui->textEdit_PROCCES->append("Gx: " + QString::number(GYROMPU[0], 'f', 2)
                                     + "  Gy: " + QString::number(GYROMPU[1], 'f', 2)
                                     + "  Gz: " + QString::number(GYROMPU[2], 'f', 2));
        break;
    case GETDATACONTROL://0xA2,
        //KP_IZQ-------------------------------------------------------------
        w.ui8[0] = datosRx[2];
        w.ui8[1] = datosRx[3];
        w.ui8[2] = datosRx[4];
        w.ui8[3] = datosRx[5];
        KP_IZQ = w.f32;
        ui->doubleSpinBox_KP_IZQ->setValue(KP_IZQ);
        ui->textEdit_PROCCES->append("KP_IZQ: " + QString::number(KP_IZQ, 'f', 5));
        //KI_IZQ-------------------------------------------------------------
        w.ui8[0] = datosRx[6];
        w.ui8[1] = datosRx[7];
        w.ui8[2] = datosRx[8];
        w.ui8[3] = datosRx[9];
        KI_IZQ = w.f32;
        ui->doubleSpinBox_KI_IZQ->setValue(KI_IZQ);
        ui->textEdit_PROCCES->append("KI_IZQ: " + QString::number(KI_IZQ, 'f', 5));
        //KD_IZQ-------------------------------------------------------------
        w.ui8[0] = datosRx[10];
        w.ui8[1] = datosRx[11];
        w.ui8[2] = datosRx[12];
        w.ui8[3] = datosRx[13];
        KD_IZQ = w.f32;
        ui->doubleSpinBox_KD_IZQ->setValue(KD_IZQ);
        ui->textEdit_PROCCES->append("KD_IZQ: " + QString::number(KD_IZQ, 'f', 5));
        //KD_IZQ-------------------------------------------------------------
        w.ui8[0] = datosRx[14];
        w.ui8[1] = datosRx[15];
        //VEL_IZQ------------------------------------------------------------
        VEL_IZQ = w.i16[0];
        ui->spinBox_VEL_IZQ->setValue(VEL_IZQ);
        ui->textEdit_PROCCES->append("VEL_IZQ: " + QString::number(VEL_IZQ));

        //KP_DER-------------------------------------------------------------
        w.ui8[0] = datosRx[16];
        w.ui8[1] = datosRx[17];
        w.ui8[2] = datosRx[18];
        w.ui8[3] = datosRx[19];
        KP_DER = w.f32;
        ui->doubleSpinBox_KP_DER->setValue(KP_DER);
        ui->textEdit_PROCCES->append("KP_DER: " + QString::number(KP_DER, 'f', 5));
        //KI_DER-------------------------------------------------------------
        w.ui8[0] = datosRx[20];
        w.ui8[1] = datosRx[21];
        w.ui8[2] = datosRx[22];
        w.ui8[3] = datosRx[23];
        KI_DER = w.f32;
        ui->doubleSpinBox_KI_DER->setValue(KI_DER);
        ui->textEdit_PROCCES->append("KI_DER: " + QString::number(KI_DER, 'f', 5));
        //KD_DER-------------------------------------------------------------
        w.ui8[0] = datosRx[24];
        w.ui8[1] = datosRx[25];
        w.ui8[2] = datosRx[26];
        w.ui8[3] = datosRx[27];
        KD_DER = w.f32;
        ui->doubleSpinBox_KD_DER->setValue(KD_DER);
        ui->textEdit_PROCCES->append("KD_DER: " + QString::number(KD_DER, 'f', 5));
        //VEL_DER-------------------------------------------------------------
        w.ui8[0] = datosRx[28];
        w.ui8[1] = datosRx[29];
        VEL_DER = w.i16[0];
        ui->spinBox_VEL_DER->setValue(VEL_DER);
        ui->textEdit_PROCCES->append("VEL_DER: " + QString::number(VEL_DER));

        //ROBOTMODE----------------------------------------------------------
        robotMode = datosRx[30];
        ui->comboBox_MODE->setCurrentIndex(robotMode);
        ui->textEdit_PROCCES->append("ROBOT MODE: " + QString::number(robotMode));
        break;
    case SETDATACONTROL://0xA3,
        if(datosRx[2]==ACK){
            str="DATA CONTROL CARGADA";
        }
        ui->textEdit_PROCCES->append(str);
    break;
    case SETFREQ://0xA4,
        w.ui8[0] = datosRx[2];
        w.ui8[1] = datosRx[3];
        w.ui8[2] = datosRx[4];
        w.ui8[3] = datosRx[5];
        e_t = w.i32;
        ui->textEdit_PROCCES->append("u(t): " + QString::number(e_t, 'f', 3));
        break;
    case GETPID:
        w.ui8[0] = datosRx[2];
        w.ui8[1] = datosRx[3];
        w.ui8[2] = datosRx[4];
        w.ui8[3] = datosRx[5];
        sensorValue = w.f32;
        ui->textEdit_PROCCES->append("sensor value: " + QString::number(sensorValue, 'f', 3));

        w.ui8[0] = datosRx[6];
        w.ui8[1] = datosRx[7];
        w.ui8[2] = datosRx[8];
        w.ui8[3] = datosRx[9];
        numerador = w.f32;
        ui->textEdit_PROCCES->append("numerador: " + QString::number(numerador, 'f', 3));

        w.ui8[0] = datosRx[10];
        w.ui8[1] = datosRx[11];
        w.ui8[2] = datosRx[12];
        w.ui8[3] = datosRx[13];
        denominador = w.f32;
        ui->textEdit_PROCCES->append("denominador: " + QString::number(denominador, 'f', 3));

        w.ui8[0] = datosRx[14];
        w.ui8[1] = datosRx[15];
        w.ui8[2] = datosRx[16];
        w.ui8[3] = datosRx[17];
        cuenta = w.f32;
        ui->textEdit_PROCCES->append("e(t): " + QString::number(cuenta, 'f', 3));

        w.ui8[0] = datosRx[18];
        w.ui8[1] = datosRx[19];
        w.ui8[2] = datosRx[20];
        w.ui8[3] = datosRx[21];
        e_t = w.f32;
        ui->textEdit_PROCCES->append("u(t): " + QString::number(e_t, 'f', 3));
    break;

    case GETALIVE://     GETALIVE=0xF0,
        if(datosRx[2]==ACK){
            contadorAlive++;
            if(source)
                    str="ALIVE BLUEPILL VIA *SERIE* RECIBIDO!!!";
            else{
                    contadorAlive++;
                    str="ALIVE BLUEPILL VIA *UDP* RECIBIDO N°: " + QString().number(contadorAlive,10);
            }
        }else{
            str= "ALIVE BLUEPILL VIA *SERIE*  NO ACK!!!";
        }
        ui->textEdit_PROCCES->append(str);
        break;
    case GETFIRMWARE://     GETFIRMWARE=0xF1
        str = "FIRMWARE:";
        for(uint8_t a=0;a<(datosRx[0]-1);a++){
            str += (QChar)datosRx[2+a];
        }
        ui->textEdit_PROCCES->append(str);

        break;

    default:
        str = str + "Comando DESCONOCIDO!!!!";
        ui->textEdit_PROCCES->append(str);
    }
}

void MainWindow::sendDataSerial(){
    uint8_t cmdId;
    _udat   w;
    //bool ok;

    unsigned char dato[256];
    unsigned char indice=0, chk=0;

    QString str="";

    dato[indice++]='U';
    dato[indice++]='N';
    dato[indice++]='E';
    dato[indice++]='R';
    dato[indice++]=0x00;
    dato[indice++]=':';
    cmdId = ui->comboBox_CMD->currentData().toInt();
    switch (cmdId) {
    case SETDATACONTROL://0xA3,
        dato[indice++] = SETDATACONTROL;
        //KP_IZQ-------------------------------------------------------------
        w.f32 = ui->doubleSpinBox_KP_IZQ->value();
        dato[indice++] = w.ui8[0];
        dato[indice++] = w.ui8[1];
        dato[indice++] = w.ui8[2];
        dato[indice++] = w.ui8[3];
        //KI_IZQ-------------------------------------------------------------
        w.f32 = ui->doubleSpinBox_KI_IZQ->value();
        dato[indice++] = w.ui8[0];
        dato[indice++] = w.ui8[1];
        dato[indice++] = w.ui8[2];
        dato[indice++] = w.ui8[3];
        //KD_IZQ-------------------------------------------------------------
        w.f32 = ui->doubleSpinBox_KD_IZQ->value();
        dato[indice++] = w.ui8[0];
        dato[indice++] = w.ui8[1];
        dato[indice++] = w.ui8[2];
        dato[indice++] = w.ui8[3];
        //VEL_IZQ------------------------------------------------------------
        w.i16[0] = ui->spinBox_VEL_IZQ->value();
        dato[indice++] = w.ui8[0];
        dato[indice++] = w.ui8[1];

        //KP_DER-------------------------------------------------------------
        w.f32 = ui->doubleSpinBox_KP_DER->value();
        dato[indice++] = w.ui8[0];
        dato[indice++] = w.ui8[1];
        dato[indice++] = w.ui8[2];
        dato[indice++] = w.ui8[3];
        //KI_DER-------------------------------------------------------------
        w.f32 = ui->doubleSpinBox_KI_DER->value();
        dato[indice++] = w.ui8[0];
        dato[indice++] = w.ui8[1];
        dato[indice++] = w.ui8[2];
        dato[indice++] = w.ui8[3];
        //KD_DER-------------------------------------------------------------
        w.f32 = ui->doubleSpinBox_KD_DER->value();
        dato[indice++] = w.ui8[0];
        dato[indice++] = w.ui8[1];
        dato[indice++] = w.ui8[2];
        dato[indice++] = w.ui8[3];
        //VEL_DER------------------------------------------------------------
        w.i16[0] = ui->spinBox_VEL_DER->value();
        dato[indice++] = w.ui8[0];
        dato[indice++] = w.ui8[1];

        dato[NBYTES]= 0x1E;
        break;
    case SETFREQ:

        break;
    case SETROBOTMODE:
        dato[indice++] = SETROBOTMODE;
        dato[indice++] = ui->comboBox_MODE->currentData().toInt();
        dato[NBYTES]= 0x03;
        break;
    case GETALIVE://0xF0
    case GETFIRMWARE://0xF1
    case GETANALOGSENSORS://0xA0
    case GETMPU://0xA1
    case GETDATACONTROL: //0xA2
    case GETPID: //0xA6
        dato[indice++]=cmdId;
        //falta implementar el envío del valor de seteo
        dato[NBYTES]=0x02;
        break;
    default:
        return;
    }
    for(int a=0 ;a<indice;a++)
        chk^=dato[a];
    dato[indice]=chk;

    if(serial->isWritable()){
        serial->write(reinterpret_cast<char *>(dato),dato[NBYTES]+PAYLOAD);
    }

    for(int i=0; i<=indice; i++){
        if(isalnum(dato[i]))
            str = str + QString("%1").arg(char(dato[i]));
        else
            str = str +"{" + QString("%1").arg(dato[i],2,16,QChar('0')) + "}";
    }

    uint16_t valor=dato[NBYTES]+PAYLOAD;
    ui->textEdit_RAW->append("INDICE ** " +QString().number(indice,10) + " **" );
    ui->textEdit_RAW->append("NUMERO DE DATOS ** " +QString().number(valor,10) + " **" );
    ui->textEdit_RAW->append("CHECKSUM ** " +QString().number(chk,16) + " **" );
    ui->textEdit_RAW->append("PC--SERIAL-->MBED ( " + str + " )");

}

void MainWindow::timeOut(){
    if(rxData.timeOut){
        rxData.timeOut--;
        if(!rxData.timeOut){
            estadoProtocolo=START;
        }
    }
}

void MainWindow::on_pushButton_ALIVE_clicked()
{
    ui->comboBox_CMD->setCurrentIndex(0);
    sendDataSerial();
}

void MainWindow::on_pushButton_OPENUDP_clicked()
{
    int Port;
    bool ok;

    if(UdpSocket1->isOpen()){
        UdpSocket1->close();
        ui->pushButton_OPENUDP->setText("OPEN UDP");
        return;
    }

    Port=ui->lineEdit_LOCALPORT->text().toInt(&ok,10);
    if(!ok || Port<=0 || Port>65535){
        QMessageBox::information(this, tr("SERVER PORT"),tr("ERRRO. Number PORT."));
        return;
    }

    try{
        UdpSocket1->abort();
        UdpSocket1->bind(Port);
        UdpSocket1->open(QUdpSocket::ReadWrite);
    }catch(...){
        QMessageBox::information(this, tr("SERVER PORT"),tr("Can't OPEN Port."));
        return;
    }
    ui->pushButton_OPENUDP->setText("CLOSE UDP");
    ui->pushButton_UDP->setEnabled(true);
    if(UdpSocket1->isOpen()){
        if(clientAddress.isNull())
            clientAddress.setAddress(ui->lineEdit_IP_REMOTA->text());
        if(puertoremoto==0)
            puertoremoto=ui->lineEdit_DEVICEPORT->text().toInt();
        UdpSocket1->writeDatagram("r", 1, clientAddress, puertoremoto);
    }
}

void MainWindow::OnUdpRxData(){
    qint64          count=0;
    unsigned char   *incomingBuffer;

    while(UdpSocket1->hasPendingDatagrams()){
        count = UdpSocket1->pendingDatagramSize();
        incomingBuffer = new unsigned char[count];
        UdpSocket1->readDatagram( reinterpret_cast<char *>(incomingBuffer), count, &RemoteAddress, &RemotePort);
    }
    if (count<=0)
        return;

    QString str="";
    for(int i=0; i<=count; i++){
        if(isalnum(incomingBuffer[i]))
            str = str + QString("%1").arg(char(incomingBuffer[i]));
        else
            str = str +"{" + QString("%1").arg(incomingBuffer[i],2,16,QChar('0')) + "}";
    }
    ui->textEdit_RAW->append("MBED-->UDP-->PC (" + str + ")");
    QString adress=RemoteAddress.toString();
    ui->textEdit_RAW->append(" adr " + adress);
    ui->lineEdit_IP_REMOTA->setText(RemoteAddress.toString().right((RemoteAddress.toString().length())-7));
    ui->lineEdit_DEVICEPORT->setText(QString().number(RemotePort,10));

    for(int i=0;i<count; i++){
        switch (estadoProtocoloUdp) {
        case START:
            if (incomingBuffer[i]=='U'){
                    estadoProtocoloUdp=HEADER_1;
                    rxDataUdp.cheksum=0;
            }
            break;
        case HEADER_1:
            if (incomingBuffer[i]=='N')
                    estadoProtocoloUdp=HEADER_2;
            else{
                    i--;
                    estadoProtocoloUdp=START;
            }
            break;
        case HEADER_2:
            if (incomingBuffer[i]=='E')
                    estadoProtocoloUdp=HEADER_3;
            else{
                    i--;
                    estadoProtocoloUdp=START;
            }
            break;
        case HEADER_3:
            if (incomingBuffer[i]=='R')
                    estadoProtocoloUdp=NBYTES;
            else{
                    i--;
                    estadoProtocoloUdp=START;
            }
            break;
        case NBYTES:
            rxDataUdp.nBytes=incomingBuffer[i];
            estadoProtocoloUdp=TOKEN;
            break;
        case TOKEN:
            if (incomingBuffer[i]==':'){
                    estadoProtocoloUdp=PAYLOAD;
                    rxDataUdp.cheksum='U'^'N'^'E'^'R'^ rxDataUdp.nBytes^':';
                    rxDataUdp.payLoad[0]=rxDataUdp.nBytes;
                    rxDataUdp.index=1;
            }
            else{
                    i--;
                    estadoProtocoloUdp=START;
            }
            break;
        case PAYLOAD:
            if (rxDataUdp.nBytes>1){
                    rxDataUdp.payLoad[rxDataUdp.index++]=incomingBuffer[i];
                    rxDataUdp.cheksum^=incomingBuffer[i];
            }
            rxDataUdp.nBytes--;
            if(rxDataUdp.nBytes==0){
                    estadoProtocoloUdp=START;
                    if(rxDataUdp.cheksum==incomingBuffer[i]){
                    decodeData(&rxDataUdp.payLoad[0],UDP);
                    }else{
                    ui->textEdit_RAW->append(" CHK DISTINTO!!!!! ");
                    }
            }
            break;

        default:
            estadoProtocoloUdp=START;
            break;
        }
    }
    delete [] incomingBuffer;

}

void MainWindow::sendDataUDP(){
    uint8_t cmdId;
    _udat w;
    unsigned char dato[256];
    unsigned char indice=0, chk=0;
    QString str;
    int puerto=0;
    //bool ok;

    dato[indice++]='U';
    dato[indice++]='N';
    dato[indice++]='E';
    dato[indice++]='R';
    dato[indice++]=0x00;
    dato[indice++]=':';
    cmdId = ui->comboBox_CMD->currentData().toInt();
    switch (cmdId) {
    case SETDATACONTROL://0xA3,
        dato[indice++] = SETDATACONTROL;
        //KP_IZQ-------------------------------------------------------------
        w.f32 = ui->doubleSpinBox_KP_IZQ->value();
        dato[indice++] = w.ui8[0];
        dato[indice++] = w.ui8[1];
        dato[indice++] = w.ui8[2];
        dato[indice++] = w.ui8[3];
        //KI_IZQ-------------------------------------------------------------
        w.f32 = ui->doubleSpinBox_KI_IZQ->value();
        dato[indice++] = w.ui8[0];
        dato[indice++] = w.ui8[1];
        dato[indice++] = w.ui8[2];
        dato[indice++] = w.ui8[3];
        //KD_IZQ-------------------------------------------------------------
        w.f32 = ui->doubleSpinBox_KD_IZQ->value();
        dato[indice++] = w.ui8[0];
        dato[indice++] = w.ui8[1];
        dato[indice++] = w.ui8[2];
        dato[indice++] = w.ui8[3];
        //VEL_IZQ------------------------------------------------------------
        w.i16[0] = ui->spinBox_VEL_IZQ->value();
        dato[indice++] = w.ui8[0];
        dato[indice++] = w.ui8[1];

        //KP_DER-------------------------------------------------------------
        w.f32 = ui->doubleSpinBox_KP_DER->value();
        dato[indice++] = w.ui8[0];
        dato[indice++] = w.ui8[1];
        dato[indice++] = w.ui8[2];
        dato[indice++] = w.ui8[3];
        //KI_DER-------------------------------------------------------------
        w.f32 = ui->doubleSpinBox_KI_DER->value();
        dato[indice++] = w.ui8[0];
        dato[indice++] = w.ui8[1];
        dato[indice++] = w.ui8[2];
        dato[indice++] = w.ui8[3];
        //KD_DER-------------------------------------------------------------
        w.f32 = ui->doubleSpinBox_KD_DER->value();
        dato[indice++] = w.ui8[0];
        dato[indice++] = w.ui8[1];
        dato[indice++] = w.ui8[2];
        dato[indice++] = w.ui8[3];
        //VEL_DER------------------------------------------------------------
        w.i16[0] = ui->spinBox_VEL_DER->value();
        dato[indice++] = w.ui8[0];
        dato[indice++] = w.ui8[1];

        dato[NBYTES]= 0x1E;
        break;
    case SETFREQ:

        break;
    case SETROBOTMODE:
        dato[indice++] = SETROBOTMODE;
        dato[indice++] = ui->comboBox_MODE->currentData().toInt();
        dato[NBYTES]= 0x03;
        break;
    case GETALIVE://0xF0
    case GETFIRMWARE://0xF1
    case GETANALOGSENSORS://0xA0
    case GETMPU://0xA1
    case GETDATACONTROL: //0xA2
    case GETPID: //0xA6
        dato[indice++]=cmdId;
        //falta implementar el envío del valor de seteo
        dato[NBYTES]=0x02;
        break;
    default:
        return;
    }

    puerto=ui->lineEdit_DEVICEPORT->text().toInt();
    puertoremoto=puerto;
    for(int a=0 ;a<indice;a++)
        chk^=dato[a];
    dato[indice]=chk;
    if(clientAddress.isNull())
        clientAddress.setAddress(ui->lineEdit_IP_REMOTA->text());
    if(puertoremoto==0)
        puertoremoto=puerto;
    if(UdpSocket1->isOpen()){
        UdpSocket1->writeDatagram(reinterpret_cast<const char *>(dato), (dato[4]+7), clientAddress, puertoremoto);
    }

    for(int i=0; i<=indice; i++){
        if(isalnum(dato[i]))
            str = str + QString("%1").arg(char(dato[i]));
        else
            str = str +"{" + QString("%1").arg(dato[i],2,16,QChar('0')) + "}";
    }
    str=str + clientAddress.toString() + "  " +  QString().number(puertoremoto,10);
    ui->textEdit_RAW->append("PC--UDP-->MBED ( " + str + " )");
}

void MainWindow::on_pushButtonClearDebugger_clicked()
{
    ui->textEdit_PROCCES->clear();
    ui->textEdit_RAW->clear();
}

void MainWindow::on_pushButton_SETEO_DE_CONTROL_clicked()
{
    ui->comboBox_CMD->setCurrentIndex(5);
    sendDataSerial();
    sendDataUDP();
}

//Control de graficas
void MainWindow::createChartACCEL()
{
    accelChart = new QChart();

    accelChart->setTitle("Valores del Aceleracion");
    accelChart->legend()->setVisible(true);
    accelChart->setAnimationOptions(QChart::AnimationOption::NoAnimation);

    accelChartView = new QChartView(accelChart);

    accelChartView->setRenderHint(QPainter::Antialiasing);

    accelLayout = new QGridLayout();

    accelLayout->addWidget(accelChartView, 0, 0);

    ui->widgetAccel->setLayout(accelLayout);

    accelXSpline = new QSplineSeries();
    accelYSpline = new QSplineSeries();
    accelZSpline = new QSplineSeries();

    for (int i = 0 ; i <= 30 ; i++)
    {
        accelXDatos.append(QPointF(i, 0));
        accelYDatos.append(QPointF(i, 0));
        accelZDatos.append(QPointF(i, 0));
    }

    accelXSpline->append(accelXDatos);
    accelYSpline->append(accelYDatos);
    accelZSpline->append(accelZDatos);

    accelXSpline->setName("ACCEL X");
    accelYSpline->setName("ACCEL Y");
    accelZSpline->setName("ACCEL Z");

    accelChart->addSeries(accelXSpline);
    accelChart->addSeries(accelYSpline);
    accelChart->addSeries(accelZSpline);

    accelChart->createDefaultAxes();
    accelChart->axes(Qt::Vertical).first()->setRange(-3, 3);
    accelChart->axes(Qt::Horizontal).first()->setRange(0, 30);
}

void MainWindow::addPointChartACCELX(uint16_t point)
{
    for (int i = 0 ; i < 30 ; i++)
    {
        accelXDatos.replace(i, QPointF(i, accelXDatos.value(i + 1).ry()));
    }

    accelXDatos.removeLast();
    accelXDatos.append(QPointF(30, point * 1.0));

    accelXSpline->clear();
    accelXSpline->append(accelXDatos);
}

void MainWindow::addPointChartACCELY(uint16_t point)
{
    for (int i = 0 ; i < 30 ; i++)
    {
        accelYDatos.replace(i, QPointF(i, accelYDatos.value(i + 1).ry()));
    }

    accelYDatos.removeLast();
    accelYDatos.append(QPointF(30, point * 1.0));

    accelYSpline->clear();
    accelYSpline->append(accelYDatos);
}

void MainWindow::addPointChartACCELZ(uint16_t point)
{
    for (int i = 0 ; i < 30 ; i++)
    {
        accelZDatos.replace(i, QPointF(i, accelZDatos.value(i + 1).ry()));
    }

    accelZDatos.removeLast();
    accelZDatos.append(QPointF(30, point * 1.0));

    accelZSpline->clear();
    accelZSpline->append(accelZDatos);
}


//Control de graficas
void MainWindow::createChartGYRO()
{
    gyroChart = new QChart();

    gyroChart->setTitle("Valores del Giroscopio");
    gyroChart->legend()->setVisible(true);
    gyroChart->setAnimationOptions(QChart::AnimationOption::NoAnimation);

    gyroChartView = new QChartView(gyroChart);

    gyroChartView->setRenderHint(QPainter::Antialiasing);

    gyroLayout = new QGridLayout();

    gyroLayout->addWidget(gyroChartView, 0, 0);

    ui->widgetGyro->setLayout(gyroLayout);

    gyroXSpline = new QSplineSeries();
    gyroYSpline = new QSplineSeries();
    gyroZSpline = new QSplineSeries();

    for (int i = 0 ; i <= 30 ; i++)
    {
        gyroXDatos.append(QPointF(i, 0));
        gyroYDatos.append(QPointF(i, 0));
        gyroZDatos.append(QPointF(i, 0));
    }

    gyroXSpline->append(gyroXDatos);
    gyroYSpline->append(gyroYDatos);
    gyroZSpline->append(gyroZDatos);

    gyroXSpline->setName("gyrox");
    gyroYSpline->setName("gyroy");
    gyroZSpline->setName("gyroz");

    gyroChart->addSeries(gyroXSpline);
    gyroChart->addSeries(gyroYSpline);
    gyroChart->addSeries(gyroZSpline);

    gyroChart->createDefaultAxes();
    gyroChart->axes(Qt::Vertical).first()->setRange(-250, 250);
    gyroChart->axes(Qt::Horizontal).first()->setRange(0, 30);
}

void MainWindow::addPointChartGYROX(uint16_t point)
{
    for (int i = 0 ; i < 30 ; i++)
    {
        gyroXDatos.replace(i, QPointF(i, gyroXDatos.value(i + 1).ry()));
    }

    gyroXDatos.removeLast();
    gyroXDatos.append(QPointF(30, point * 1.0));

    gyroXSpline->clear();
    gyroXSpline->append(gyroXDatos);
}

void MainWindow::addPointChartGYROY(uint16_t point)
{
    for (int i = 0 ; i < 30 ; i++)
    {
        gyroYDatos.replace(i, QPointF(i, gyroYDatos.value(i + 1).ry()));
    }

    gyroYDatos.removeLast();
    gyroYDatos.append(QPointF(30, point * 1.0));

    gyroYSpline->clear();
    gyroYSpline->append(gyroYDatos);
}

void MainWindow::addPointChartGYROZ(uint16_t point)
{
    for (int i = 0 ; i < 30 ; i++)
    {
        gyroZDatos.replace(i, QPointF(i, gyroZDatos.value(i + 1).ry()));
    }

    gyroZDatos.removeLast();
    gyroZDatos.append(QPointF(30, point * 1.0));

    gyroZSpline->clear();
    gyroZSpline->append(gyroZDatos);
}
