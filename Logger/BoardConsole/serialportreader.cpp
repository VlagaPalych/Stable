/****************************************************************************
**
** Copyright (C) 2013 Laszlo Papp <lpapp@kde.org>
** Contact: http://www.qt-project.org/legal
**
** This file is part of the QtSerialPort module of the Qt Toolkit.
**
** $QT_BEGIN_LICENSE:LGPL$
** Commercial License Usage
** Licensees holding valid commercial Qt licenses may use this file in
** accordance with the commercial license agreement provided with the
** Software or, alternatively, in accordance with the terms contained in
** a written agreement between you and Digia.  For licensing terms and
** conditions see http://qt.digia.com/licensing.  For further information
** use the contact form at http://qt.digia.com/contact-us.
**
** GNU Lesser General Public License Usage
** Alternatively, this file may be used under the terms of the GNU Lesser
** General Public License version 2.1 as published by the Free Software
** Foundation and appearing in the file LICENSE.LGPL included in the
** packaging of this file.  Please review the following information to
** ensure the GNU Lesser General Public License version 2.1 requirements
** will be met: http://www.gnu.org/licenses/old-licenses/lgpl-2.1.html.
**
** In addition, as a special exception, Digia gives you certain additional
** rights.  These rights are described in the Digia Qt LGPL Exception
** version 1.1, included in the file LGPL_EXCEPTION.txt in this package.
**
** GNU General Public License Usage
** Alternatively, this file may be used under the terms of the GNU
** General Public License version 3.0 as published by the Free Software
** Foundation and appearing in the file LICENSE.GPL included in the
** packaging of this file.  Please review the following information to
** ensure the GNU General Public License version 3.0 requirements will be
** met: http://www.gnu.org/copyleft/gpl.html.
**
**
** $QT_END_LICENSE$
**
****************************************************************************/

#include "serialportreader.h"

#include <QCoreApplication>
#include <QDebug>
#include "commands.h"

QT_USE_NAMESPACE

SerialPortReader::SerialPortReader(QSerialPort *serialPort, QString fileName, QObject *parent)
: QObject(parent)
, m_serialPort(serialPort)
, m_standardOutput(stdout)
{
	log = NULL;
	logStream = NULL;
	paramsBitMask = 0;

	connect(m_serialPort, SIGNAL(readyRead()), SLOT(handleReadyRead()));
	connect(m_serialPort, SIGNAL(error(QSerialPort::SerialPortError)), SLOT(handleError(QSerialPort::SerialPortError)));

	setFile(fileName);
}

SerialPortReader::~SerialPortReader()
{
	if (log) {
		log->close();
		delete log;
		delete logStream;
	}
}
void SerialPortReader::setParamsBitMask(quint32 bitMask) {
	paramsBitMask = bitMask;
	messageSize = 2;
	if (paramsBitMask & BIT_ACCEL_X)
		messageSize += 4;
	if (paramsBitMask & BIT_ACCEL_Y)
		messageSize += 4;
	if (paramsBitMask & BIT_ACCEL_Z)
		messageSize += 4;
	if (paramsBitMask & BIT_GYRO_X)
		messageSize += 4;
	if (paramsBitMask & BIT_GYRO_Y)
		messageSize += 4;
	if (paramsBitMask & BIT_GYRO_Z)
		messageSize += 4;
	if (paramsBitMask & BIT_COMPASS_X)
		messageSize += 4;
	if (paramsBitMask & BIT_COMPASS_Y)
		messageSize += 4;
	if (paramsBitMask & BIT_COMPASS_Z)
		messageSize += 4;
	if (paramsBitMask & BIT_MPL_EULER_X)
		messageSize += 4;
	if (paramsBitMask & BIT_MPL_EULER_Y)
		messageSize += 4;
	if (paramsBitMask & BIT_MPL_EULER_Z)
		messageSize += 4;
	if (paramsBitMask & BIT_DMP_EULER_X)
		messageSize += 4;
	if (paramsBitMask & BIT_DMP_EULER_Y)
		messageSize += 4;
	if (paramsBitMask & BIT_DMP_EULER_Z)
		messageSize += 4;
	if (paramsBitMask & BIT_MINE_EULER_X)
		messageSize += 4;
	if (paramsBitMask & BIT_MINE_EULER_Y)
		messageSize += 4;
	if (paramsBitMask & BIT_MINE_EULER_Z)
		messageSize += 4;
	if (paramsBitMask & BIT_PWM1)
		messageSize += 2;
	if (paramsBitMask & BIT_PWM2)
		messageSize += 2;
	if (paramsBitMask & BIT_FREQ1)
		messageSize += 2;
	if (paramsBitMask & BIT_FREQ2)
		messageSize += 2;
	if (paramsBitMask & BIT_F)
		messageSize += 4;
}


void SerialPortReader::analyseFileName(QString fileName) {
	if (fileName.isEmpty()) {
		log = NULL;
		logStream = NULL;
	}
	else {
		log = new QFile(fileName);
		log->open(QFile::WriteOnly);
		logStream = new QTextStream(log);
	}

}

void SerialPortReader::setFile(QString fileName) {
	if (log) {
		log->close();
		delete log;
		delete logStream;
	}
	analyseFileName(fileName);
}

qint32 errorCounter = 0;

void SerialPortReader::handleReadyRead()
{
	m_readData.append(m_serialPort->readAll());
	while (m_readData.size() >= messageSize) {
		QByteArray msgByteArray = m_readData.mid(0, messageSize);
		if (Message_FromByteArray(msgByteArray, paramsBitMask, &msg)) {	
			if (logStream) {
				(*logStream) << msg.accel[0] << ' ' << msg.accel[1] << ' ' << msg.accel[2] << ' '
					<< msg.mine_euler[0] << ' ' << msg.mine_euler[1] << ' '
					<< msg.mpl_euler[0] << ' ' << msg.mpl_euler[1] << endl;
				qDebug() << msg.accel[0] << ' ' << msg.accel[1] << ' ' << msg.accel[2] << ' '
					<< msg.mine_euler[0] << ' ' << msg.mine_euler[1] << ' '
					<< msg.mpl_euler[0] << ' ' << msg.mpl_euler[1] << endl;
			}
			Q_EMIT freshMessage(&msg);
			m_readData = m_readData.remove(0, messageSize);
		}
		else {
			errorCounter++;
			qDebug() << "errors=" << errorCounter << endl;
			m_readData.remove(0, 1);
		}
		
	}
}


void SerialPortReader::handleError(QSerialPort::SerialPortError serialPortError)
{
	if (serialPortError == QSerialPort::ReadError) {
		m_standardOutput << QObject::tr("An I/O error occurred while reading the data from port %1, error: %2").arg(m_serialPort->portName()).arg(m_serialPort->errorString()) << endl;
		QCoreApplication::exit(1);
	}
}


