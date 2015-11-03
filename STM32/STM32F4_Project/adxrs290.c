#include "adxrs290.h"
#include "adxl345.h"
#include "stm32f4xx.h"

//-------------------------------------------------------------------
// COMMON
//-------------------------------------------------------------------

uint8_t SPI2_Busy = 0;
SPI2_Using SPI2_curUsing = NONE;

uint8_t adxrs290_regs[ADXRS290_DATA_SIZE*2] = {0x08, 0x09, 0x0a, 0x0b, 0x0c, 0x0d};

float adxrs290_filterCfs[];// = {-0.00116826272195121, -0.000117893990554694, -0.000123700754869243, -0.000129760939053369, -0.000135752851730653, -0.000141907870398519, -0.000147886887616796, -0.000153946718722314, -0.000159774627377435, -0.000165752415866396, -0.000171625265435308, -0.000177916066597755, -0.000184242488024093, -0.000190978599163771, -0.000196628277616392, -0.000200694025939228, -0.000208468250159458, -0.000213626005994046, -0.000219479868890997, -0.000225015845003470, -0.000230514888268066, -0.000235802332464375, -0.000240994721225324, -0.000246069278862743, -0.000251025946672788, -0.000255845309768882, -0.000260344121310948, -0.000264528714277783, -0.000268289489190999, -0.000272240364136562, -0.000276085635404284, -0.000278779369961839, -0.000281926357528071, -0.000284326658256693, -0.000286533525320535, -0.000288302646403253, -0.000289766716246888, -0.000290801713525966, -0.000291418539003650, -0.000291537075433414, -0.000291156155907540, -0.000290335067266280, -0.000289088148061122, -0.000287405922672860, -0.000284923578666196, -0.000281916588689285, -0.000278609071409165, -0.000274343872654109, -0.000269718551275990, -0.000264335280228550, -0.000258357506109918, -0.000251629111759172, -0.000244211960951397, -0.000236047833760111, -0.000227185372214043, -0.000217584571823890, -0.000207184977127340, -0.000195917325024293, -0.000183789841421168, -0.000170976394733371, -0.000157217311323976, -0.000142495491176240, -0.000127062325045897, -0.000110541427569879, -9.31748199130345e-05, -7.47928409766891e-05, -5.54979772942300e-05, -3.52084172312074e-05, -1.39495427397473e-05, 8.36028782703392e-06, 3.17169062282784e-05, 5.61137987915576e-05, 8.15144153660527e-05, 0.000107978820399827, 0.000135617247429969, 0.000164261762051521, 0.000194012288058499, 0.000224978633779860, 0.000256962206118745, 0.000290150740877119, 0.000324427318443776, 0.000359902716201922, 0.000396531429830177, 0.000434362318126741, 0.000473341852204640, 0.000513492995783737, 0.000554831135321481, 0.000597403969196572, 0.000641159816110910, 0.000686067413032707, 0.000732253383012984, 0.000779606364133484, 0.000828113619277541, 0.000877887556940219, 0.000928807919593966, 0.000980973892288927, 0.00103429588274482, 0.00108881293594153, 0.00114447838618901, 0.00120133737831862, 0.00125936607323902, 0.00131854961512164, 0.00137884520873752, 0.00144029728414793, 0.00150288514591874, 0.00156651757654523, 0.00163128728585519, 0.00169714879023139, 0.00176403570040761, 0.00183200446913589, 0.00190095668786337, 0.00197094285732181, 0.00204190618357576, 0.00211386427637368, 0.00218675085036298, 0.00226055818093853, 0.00233526719432960, 0.00241087326678898, 0.00248730071391498, 0.00256455798957963, 0.00264265786086075, 0.00272148444580798, 0.00280105767846911, 0.00288135663014469, 0.00296231055016595, 0.00304394860268665, 0.00312618424890889, 0.00320901563296091, 0.00329237346060195, 0.00337626106118999, 0.00346062801689431, 0.00354543330083973, 0.00363063346990356, 0.00371623174860829, 0.00380214196016913, 0.00388831480614718, 0.00397477415158878, 0.00406142709273342, 0.00414825002843658, 0.00423521747638886, 0.00432224470792161, 0.00440932978958172, 0.00449640352660456, 0.00458345458517064, 0.00467040637238258, 0.00475723110658680, 0.00484388995638494, 0.00493033250457231, 0.00501648552959640, 0.00510235460856114, 0.00518788370801815, 0.00527298906429789, 0.00535767334663056, 0.00544185924988821, 0.00552550251032150, 0.00560858506804166, 0.00569103349840662, 0.00577282444057752, 0.00585388477606505, 0.00593419912654524, 0.00601370408117498, 0.00609234996299459, 0.00617010860687476, 0.00624695044994782, 0.00632278682930448, 0.00639759859847579, 0.00647135877943608, 0.00654398606463250, 0.00661547688054606, 0.00668577726951352, 0.00675483212048534, 0.00682261830603607, 0.00688907866516137, 0.00695419764839725, 0.00701790912208956, 0.00708019753354154, 0.00714102876498466, 0.00720034330565265, 0.00725810531213738, 0.00731431326803450, 0.00736890311743815, 0.00742184416342981, 0.00747313206561224, 0.00752269614690386, 0.00757052539269782, 0.00761659355544603, 0.00766086421097149, 0.00770332232840869, 0.00774392289201543, 0.00778266710675782, 0.00781950393388064, 0.00785441182777756, 0.00788739488459493, 0.00791841978888290, 0.00794744963585689, 0.00797449577256499, 0.00799952574407568, 0.00802250680761024, 0.00804346143366054, 0.00806235347726042, 0.00807917707713300, 0.00809392206033690, 0.00810657283755571, 0.00811713459714587, 0.00812557568055866, 0.00813192305297812, 0.00813616469148631, 0.00813827456483562, 0.00813827456483562, 0.00813616469148631, 0.00813192305297812, 0.00812557568055866, 0.00811713459714587, 0.00810657283755571, 0.00809392206033690, 0.00807917707713300, 0.00806235347726042, 0.00804346143366054, 0.00802250680761024, 0.00799952574407568, 0.00797449577256499, 0.00794744963585689, 0.00791841978888290, 0.00788739488459493, 0.00785441182777756, 0.00781950393388064, 0.00778266710675782, 0.00774392289201543, 0.00770332232840869, 0.00766086421097149, 0.00761659355544603, 0.00757052539269782, 0.00752269614690386, 0.00747313206561224, 0.00742184416342981, 0.00736890311743815, 0.00731431326803450, 0.00725810531213738, 0.00720034330565265, 0.00714102876498466, 0.00708019753354154, 0.00701790912208956, 0.00695419764839725, 0.00688907866516137, 0.00682261830603607, 0.00675483212048534, 0.00668577726951352, 0.00661547688054606, 0.00654398606463250, 0.00647135877943608, 0.00639759859847579, 0.00632278682930448, 0.00624695044994782, 0.00617010860687476, 0.00609234996299459, 0.00601370408117498, 0.00593419912654524, 0.00585388477606505, 0.00577282444057752, 0.00569103349840662, 0.00560858506804166, 0.00552550251032150, 0.00544185924988821, 0.00535767334663056, 0.00527298906429789, 0.00518788370801815, 0.00510235460856114, 0.00501648552959640, 0.00493033250457231, 0.00484388995638494, 0.00475723110658680, 0.00467040637238258, 0.00458345458517064, 0.00449640352660456, 0.00440932978958172, 0.00432224470792161, 0.00423521747638886, 0.00414825002843658, 0.00406142709273342, 0.00397477415158878, 0.00388831480614718, 0.00380214196016913, 0.00371623174860829, 0.00363063346990356, 0.00354543330083973, 0.00346062801689431, 0.00337626106118999, 0.00329237346060195, 0.00320901563296091, 0.00312618424890889, 0.00304394860268665, 0.00296231055016595, 0.00288135663014469, 0.00280105767846911, 0.00272148444580798, 0.00264265786086075, 0.00256455798957963, 0.00248730071391498, 0.00241087326678898, 0.00233526719432960, 0.00226055818093853, 0.00218675085036298, 0.00211386427637368, 0.00204190618357576, 0.00197094285732181, 0.00190095668786337, 0.00183200446913589, 0.00176403570040761, 0.00169714879023139, 0.00163128728585519, 0.00156651757654523, 0.00150288514591874, 0.00144029728414793, 0.00137884520873752, 0.00131854961512164, 0.00125936607323902, 0.00120133737831862, 0.00114447838618901, 0.00108881293594153, 0.00103429588274482, 0.000980973892288927, 0.000928807919593966, 0.000877887556940219, 0.000828113619277541, 0.000779606364133484, 0.000732253383012984, 0.000686067413032707, 0.000641159816110910, 0.000597403969196572, 0.000554831135321481, 0.000513492995783737, 0.000473341852204640, 0.000434362318126741, 0.000396531429830177, 0.000359902716201922, 0.000324427318443776, 0.000290150740877119, 0.000256962206118745, 0.000224978633779860, 0.000194012288058499, 0.000164261762051521, 0.000135617247429969, 0.000107978820399827, 8.15144153660527e-05, 5.61137987915576e-05, 3.17169062282784e-05, 8.36028782703392e-06, -1.39495427397473e-05, -3.52084172312074e-05, -5.54979772942300e-05, -7.47928409766891e-05, -9.31748199130345e-05, -0.000110541427569879, -0.000127062325045897, -0.000142495491176240, -0.000157217311323976, -0.000170976394733371, -0.000183789841421168, -0.000195917325024293, -0.000207184977127340, -0.000217584571823890, -0.000227185372214043, -0.000236047833760111, -0.000244211960951397, -0.000251629111759172, -0.000258357506109918, -0.000264335280228550, -0.000269718551275990, -0.000274343872654109, -0.000278609071409165, -0.000281916588689285, -0.000284923578666196, -0.000287405922672860, -0.000289088148061122, -0.000290335067266280, -0.000291156155907540, -0.000291537075433414, -0.000291418539003650, -0.000290801713525966, -0.000289766716246888, -0.000288302646403253, -0.000286533525320535, -0.000284326658256693, -0.000281926357528071, -0.000278779369961839, -0.000276085635404284, -0.000272240364136562, -0.000268289489190999, -0.000264528714277783, -0.000260344121310948, -0.000255845309768882, -0.000251025946672788, -0.000246069278862743, -0.000240994721225324, -0.000235802332464375, -0.000230514888268066, -0.000225015845003470, -0.000219479868890997, -0.000213626005994046, -0.000208468250159458, -0.000200694025939228, -0.000196628277616392, -0.000190978599163771, -0.000184242488024093, -0.000177916066597755, -0.000171625265435308, -0.000165752415866396, -0.000159774627377435, -0.000153946718722314, -0.000147886887616796, -0.000141907870398519, -0.000135752851730653, -0.000129760939053369, -0.000123700754869243, -0.000117893990554694, -0.00116826272195121};

void All_NSS_High() {
    Accel_NSS_High();
    ARS1_NSS_High();
    ARS2_NSS_High();
}

void SPI2_SensorsPoll(void) { 
    SPI2_curUsing = NONE;
    if (GPIOA->IDR & (1 << ACCEL_INT1)) {
        // Accel DATA_READY
        EXTI->SWIER |= EXTI_SWIER_SWIER1;
    } else if (GPIOD->IDR & (1 << ARS1_EXTI)) {
        // ARS1 DATA_READY
        EXTI->SWIER |= EXTI_SWIER_SWIER10;
    } else if (GPIOE->IDR & (1 << ARS2_EXTI)) {
        // ARS2 DATA_READY
        EXTI->SWIER |= EXTI_SWIER_SWIER15;
    }
}

void EXTI15_10_IRQHandler() {
    if (EXTI->PR & EXTI_PR_PR10) {
        EXTI->PR = EXTI_PR_PR10;
         
        ARS1_GetData();
    } else if (EXTI->PR & EXTI_PR_PR15) {
        EXTI->PR = EXTI_PR_PR15;
        
        ARS2_GetData();
    }
}

void SPI2_IRQHandler() {
    uint8_t i = 0;
    uint16_t tmp = 0;
    
    if (SPI2->SR & SPI_SR_RXNE) { 
        if (SPI2_curUsing == ARS1_USING) {
            ARS1_NSS_High();        
            tmp = SPI2->DR;
            ars1_rawData[ars1_spiIndex] = (uint8_t)(tmp & 0xff);
            ars1_spiIndex++;
            
            if (ars1_spiIndex < ADXRS290_DATA_SIZE*2) {
                ARS1_NSS_Low();
                tmp = (adxrs290_regs[ars1_spiIndex] | 0x80) << 8;
                SPI2->DR = tmp;
            } else {
                SPI2->CR2 &= ~SPI_CR2_RXNEIE;
                //SPI2->CR1 &= ~SPI_CR1_SPE;
                
                ars1_data[0] = (ars1_rawData[1] << 8) | ars1_rawData[0];
                ars1_data[1] = (ars1_rawData[3] << 8) | ars1_rawData[2];
                ars1_data[2] = ((ars1_rawData[5] & 0x0f) << 8) | ars1_rawData[4];
                
                
                for (i = 0; i < ADXRS290_DATA_SIZE-1; i++) {
                    ars1_termoData[i] = ars1_data[i] - ars1_termoCf[i] * ars1_data[2];
                    //ars1_data[i] -= ars1_offset[i];                     // subract pre-calibrated offset
                    ars1_history[i][ars1_historyIndex] = ars1_data[i];  // recording history for future filtration
                }
                
                ars1_historyIndex++;
                if (ars1_historyIndex >= ADXRS290_FILTER_SIZE) {
                    ars1_lowpassReady = 1;
                    ars1_historyIndex = 0;
                }    

                ars1_processIndex++;
                if (ars1_processIndex == ars1_processNumber) {
                    ars1_processIndex = 0;
                    ars1_curHistoryIndex = ars1_historyIndex - 1;
                    if (lowpassOn && ars1_lowpassReady) {
                        ars1_doProcess = 1;
                    }
                }
                
                SPI2_SensorsPoll();
            }
        } else if (SPI2_curUsing == ARS2_USING) {
            ARS2_NSS_High();        
            tmp = SPI2->DR;
            ars2_rawData[ars2_spiIndex] = (uint8_t)(tmp & 0xff);
            ars2_spiIndex++;
            
            if (ars2_spiIndex < ADXRS290_DATA_SIZE*2) {
                ARS2_NSS_Low();
                tmp = (adxrs290_regs[ars2_spiIndex] | 0x80) << 8;
                SPI2->DR = tmp;
            } else {
                SPI2->CR2 &= ~SPI_CR2_RXNEIE;
                //SPI2->CR1 &= ~SPI_CR1_SPE;
                
                ars2_data[0] = (ars2_rawData[1] << 8) | ars2_rawData[0];
                ars2_data[1] = (ars2_rawData[3] << 8) | ars2_rawData[2];
                ars2_data[2] = ((ars2_rawData[5] & 0x0f) << 8) | ars2_rawData[4];
                
                for (i = 0; i < ADXRS290_DATA_SIZE-1; i++) {
                    ars2_termoData[i] = ars2_data[i] - ars2_termoCf[i] * ars2_data[2]; // termocompensation
                    //ars2_data[i] -= ars2_offset[i];                     // subract pre-calibrated offset
                    ars2_history[i][ars2_historyIndex] = ars2_data[i];  // recording history for future filtration
                }
                
                ars2_historyIndex++;
                if (ars2_historyIndex >= ADXRS290_FILTER_SIZE) {
                    ars2_lowpassReady = 1;
                    ars2_historyIndex = 0;
                }    

                ars2_processIndex++;
                if (ars2_processIndex == ars2_processNumber) {
                    ars2_processIndex = 0;
                    ars2_curHistoryIndex = ars2_historyIndex - 1;
                    if (lowpassOn && ars2_lowpassReady) {
                        ars2_doProcess = 1;
                    }
                }
                
                SPI2_SensorsPoll();
            }
        }
    }
}

//-------------------------------------------------------------------
// ARS1
//-------------------------------------------------------------------

uint8_t ARS1_NSS    = 9;    // PD
uint8_t ARS1_VDD    = 8;    // PD
uint8_t ARS1_EXTI   = 10;   // PD

uint8_t ars1_spiIndex = 0;
uint8_t ars1_rawData[ADXRS290_DATA_SIZE*2];
int16_t ars1_data[ADXRS290_DATA_SIZE]; 
float ars1_termoData[ADXRS290_DATA_SIZE-1];
float ars1_filteredData[ADXRS290_DATA_SIZE-1];
float ars1_angleRate[ADXRS290_DATA_SIZE-1];

uint8_t ars1_calibrationOn  = 0;
uint32_t ars1_calibrIndex   = 0;
uint32_t ars1_calibrNumber  = 0;
float ars1_offset[ADXRS290_DATA_SIZE-1];
float ars1_sum[ADXRS290_DATA_SIZE-1];

int16_t ars1_history[ADXRS290_DATA_SIZE-1][HISTORY_SIZE];
uint16_t ars1_historyIndex       = 0;
uint16_t ars1_curHistoryIndex    = 0;
uint8_t ars1_processIndex       = 0;
uint8_t ars1_processNumber      = 42;
uint8_t ars1_lowpassReady       = 0;
uint8_t ars1_doProcess          = 0;

float ars1_termoCf[ADXRS290_DATA_SIZE-1] = {-0.04946, -1.761};

void ARS1_VDD_Init() {
    GPIOD->MODER |= 1 << ARS1_VDD*2;
    GPIOD->BSRRL |= 1 << ARS1_VDD;
}

void ARS1_NSS_Init() {
    GPIOD->MODER |= 1 << ARS1_NSS*2;
    GPIOD->OSPEEDR |= 3 << ARS1_NSS*2;
}

void ARS1_NSS_Low() {
    GPIOD->BSRRH |= 1 << ARS1_NSS;
}

void ARS1_NSS_High() {
    GPIOD->BSRRL |= 1 << ARS1_NSS;
}

void ARS1_Init() {
    uint8_t ars1_test = 0;
    
    ARS1_NSS_Init();
    ARS1_NSS_High();
    
    // Turn measurement and termometer on
    ARS1_NSS_Low();
    SPI2_Write(0x10, 0x03);
    ARS1_NSS_High();
    
    ARS1_NSS_Low();
    ars1_test = SPI2_Read(0x10);
    ARS1_NSS_High();
    
    // Turn DATA_READY interrupt on
    ARS1_NSS_Low();
    SPI2_Write(0x12, 0x01);
    ARS1_NSS_High();
    
    ARS1_NSS_Low();
    ars1_test = SPI2_Read(0x12);
    ARS1_NSS_High();
    
    NVIC_EnableIRQ(SPI2_IRQn);
}

void ARS1_EXTI_Init() {
    SYSCFG->EXTICR[2] |= SYSCFG_EXTICR3_EXTI10_PD;
    
    EXTI->RTSR 	|= EXTI_FTSR_TR10; 
    EXTI->IMR 	|= EXTI_IMR_MR10;
    NVIC_SetPriority(EXTI15_10_IRQn, 0x06);
    NVIC_EnableIRQ(EXTI15_10_IRQn);
}

void ARS1_Calibr() {
    uint8_t i = 0;
    for (i = 0; i < ADXRS290_DATA_SIZE; i++) {
        ars1_sum[i] = 0;
    }
    ars1_calibrIndex = 0;
    ars1_calibrNumber = 100; // TODO: make number of calibration samples clear
    ars1_calibrationOn = 1;
}

void ARS1_GetData() {
    uint8_t i = 0;
    
    if (SPI2_curUsing == NONE) {
        SPI2_curUsing = ARS1_USING;
        
        ars1_spiIndex = 0;
        ARS1_NSS_Low();
        SPI2->CR2 |= SPI_CR2_RXNEIE;
        //SPI2->CR1 |= SPI_CR1_SPE;
        
        SPI2->DR = (adxrs290_regs[0] | 0x80) << 8;
        
//        for (i = 0; i < ADXRS290_DATA_SIZE*2; i++) {
//            ARS1_NSS_Low();
//            ars1_rawData[i] = SPI2_Read(adxrs290_regs[i]);
//            ARS1_NSS_High();
//        }
//        All_NSS_High();
//        
//        ars1_data[0] = (ars1_rawData[1] << 8) | ars1_rawData[0];
//        ars1_data[1] = (ars1_rawData[3] << 8) | ars1_rawData[2];
//        ars1_data[2] = ((ars1_rawData[5] & 0x0f) << 8) | ars1_rawData[4];
//        
//        if (ars1_calibrationOn) {
//            for (i = 0; i < ADXRS290_DATA_SIZE-1; i++) {
//                ars1_sum[i] += ars1_data[i];
//            }
//            ars1_calibrIndex++;          
//            if (ars1_calibrIndex == ars1_calibrNumber) {
//                for (i = 0; i < ADXRS290_DATA_SIZE-1; i++) {
//                    ars1_offset[i] = (int16_t)(ars1_sum[i] / ars1_calibrNumber);
//                }
//                ars1_calibrationOn = 0;
//            }
//        }
//        for (i = 0; i < ADXRS290_DATA_SIZE-1; i++) {
//            //ars1_data[i] -= ars1_offset[i];                     // subract pre-calibrated offset
//            ars1_history[i][ars1_historyIndex] = ars1_data[i];  // recording history for future filtration
//        }
//        
//        ars1_historyIndex++;
//        if (ars1_historyIndex >= ADXRS290_FILTER_SIZE) {
//            ars1_lowpassReady = 1;
//            ars1_historyIndex = 0;
//        }    

//        ars1_processIndex++;
//        if (ars1_processIndex == ars1_processNumber) {
//            ars1_processIndex = 0;
//            ars1_curHistoryIndex = ars1_historyIndex - 1;
//            if (lowpassOn && ars1_lowpassReady) {
//                ars1_doProcess = 1;
//            }
//        }   
        
       // SPI2_SensorsPoll();
    }
}

//-------------------------------------------------------------------
// ARS2
//-------------------------------------------------------------------

uint8_t ARS2_NSS    = 10; // PB
uint8_t ARS2_VDD    = 14; // PE
uint8_t ARS2_EXTI   = 15; // PE

uint8_t ars2_rawData[ADXRS290_DATA_SIZE*2];
int16_t ars2_data[ADXRS290_DATA_SIZE];  
float ars2_termoData[ADXRS290_DATA_SIZE-1];
float ars2_filteredData[ADXRS290_DATA_SIZE-1];
float ars2_angleRate[ADXRS290_DATA_SIZE-1];
uint8_t ars2_spiIndex = 0;

uint8_t ars2_calibrationOn = 0;
float ars2_offset[ADXRS290_DATA_SIZE-1];
float ars2_sum[ADXRS290_DATA_SIZE-1];
uint32_t ars2_calibrIndex = 0;
uint32_t ars2_calibrNumber = 0;

int16_t ars2_history[ADXRS290_DATA_SIZE-1][HISTORY_SIZE];
uint16_t ars2_historyIndex       = 0;
uint16_t ars2_curHistoryIndex    = 0;
uint8_t ars2_processIndex       = 0;
uint8_t ars2_processNumber      = 42;
uint8_t ars2_lowpassReady       = 0;
uint8_t ars2_doProcess          = 0;

float ars2_termoCf[ADXRS290_DATA_SIZE-1] = {0.2236, -1.309};

void ARS2_VDD_Init() {
    GPIOE->MODER |= 1 << ARS2_VDD*2;
    GPIOE->BSRRL |= 1 << ARS2_VDD;
}

void ARS2_NSS_Init() {
    GPIOB->MODER    |= 1 << ARS2_NSS*2;
    GPIOB->OSPEEDR  |= 3 << ARS2_NSS*2;
}

void ARS2_NSS_Low() {
    GPIOB->BSRRH |= 1 << ARS2_NSS;
}

void ARS2_NSS_High() {
    GPIOB->BSRRL |= 1 << ARS2_NSS;
}

void ARS2_Init() {
    uint8_t ars2_test = 0;
    
    ARS2_VDD_Init();
    ARS2_NSS_Init();
    ARS2_NSS_High();
    
    // Turn measurement and termometer on
    ARS2_NSS_Low();
    SPI2_Write(0x10, 0x03);
    ARS2_NSS_High();
    
    ARS2_NSS_Low();
    ars2_test = SPI2_Read(0x10);
    ARS2_NSS_High();
    
    // Turn DATA_READY interrupt on
    ARS2_NSS_Low();
    SPI2_Write(0x12, 0x01);
    ARS2_NSS_High();
    
    ARS2_NSS_Low();
    ars2_test = SPI2_Read(0x12);
    ARS2_NSS_High();
}

void ARS2_EXTI_Init() { // PE15
    SYSCFG->EXTICR[3] |= SYSCFG_EXTICR4_EXTI15_PE;
    
    EXTI->RTSR 	|= EXTI_FTSR_TR15; 
    EXTI->IMR 	|= EXTI_IMR_MR15;
    NVIC_SetPriority(EXTI15_10_IRQn, 0x06);
    NVIC_EnableIRQ(EXTI15_10_IRQn);
}

void ARS2_Calibr() {
    uint8_t i = 0;
    for (i = 0; i < ADXRS290_DATA_SIZE; i++) {
        ars2_sum[i] = 0;
    }
    ars2_calibrIndex = 0;
    ars2_calibrNumber = 100; // TODO: make number of calibration samples clear
    ars2_calibrationOn = 1;
}

void ARS2_GetData() {
    uint8_t i = 0;
    
    if (SPI2_curUsing == NONE) {
        SPI2_curUsing = ARS2_USING;
        
        ars2_spiIndex = 0;
        ARS2_NSS_Low();
        SPI2->CR2 |= SPI_CR2_RXNEIE;
        //SPI2->CR1 |= SPI_CR1_SPE;
        
        SPI2->DR = (adxrs290_regs[0] | 0x80) << 8;
    }
    
//    if (SPI2_Busy == 0/*(SPI2->SR & SPI_SR_BSY) == 0*/) {
//        SPI2_Busy = 1;
//        
//        for (i = 0; i < ADXRS290_DATA_SIZE*2; i++) {
//            ARS2_NSS_Low();
//            ars2_rawData[i] = SPI2_Read(adxrs290_regs[i]);
//            ARS2_NSS_High();
//        }
//        All_NSS_High();
//        
//        ars2_data[0] = (ars2_rawData[1] << 8) | ars2_rawData[0];
//        ars2_data[1] = (ars2_rawData[3] << 8) | ars2_rawData[2];
//        ars2_data[2] = ((ars2_rawData[5] & 0x0f) << 8) | ars2_rawData[4];
//        
//        if (ars2_calibrationOn) {
//            for (i = 0; i < ADXRS290_DATA_SIZE-1; i++) {
//                ars2_sum[i] += ars2_data[i];
//            }
//            ars2_calibrIndex++;          
//            if (ars2_calibrIndex == ars2_calibrNumber) {
//                for (i = 0; i < ADXRS290_DATA_SIZE-1; i++) {
//                    ars2_offset[i] = (int16_t)(ars2_sum[i] / ars2_calibrNumber);
//                }
//                ars2_calibrationOn = 0;
//            }
//        }
//        for (i = 0; i < ADXRS290_DATA_SIZE-1; i++) {
//            //ars2_data[i] -= ars2_offset[i];                     // subract pre-calibrated offset
//            ars2_history[i][ars2_historyIndex] = ars1_data[i];  // recording history for future filtration
//        }
//        
//        ars2_historyIndex++;
//        if (ars2_historyIndex >= ADXRS290_FILTER_SIZE) {
//            ars2_lowpassReady = 1;
//            ars2_historyIndex = 0;
//        }    

//        ars2_processIndex++;
//        if (ars2_processIndex == ars2_processNumber) {
//            ars2_processIndex = 0;
//            ars2_curHistoryIndex = ars2_historyIndex - 1;
//            if (lowpassOn && ars2_lowpassReady) {
//                ars2_doProcess = 1;
//            }
//        }
//        
//        SPI2_SensorsPoll();
//    }
}

//void ARS2_DMA_Init() {
//    NVIC_SetPriority(DMA1_Stream3_IRQn, 0x05);    
//    NVIC_EnableIRQ(DMA1_Stream3_IRQn);
//    NVIC_EnableIRQ(DMA1_Stream4_IRQn);
//    
//    DMA1->HIFCR = DMA_HIFCR_CFEIF4;
//    
//    DMA1_Stream4->CR    = 0;
//    DMA1_Stream3->CR    = 0;
//    
//    DMA1_Stream3->PAR   = (uint32_t)&(SPI2->DR);
//    DMA1_Stream3->M0AR  = (uint32_t)(ars2_rawData /*+ ars2_dma_status*2*/);
//    DMA1_Stream3->NDTR  = 2;
//    DMA1_Stream3->CR    |= DMA_SxCR_MINC | DMA_SxCR_PSIZE_0 | DMA_SxCR_MSIZE_0 |
//                                DMA_SxCR_TCIE | DMA_SxCR_PL | DMA_SxCR_EN; 
//    
//    DMA1_Stream4->PAR   = (uint32_t)&(SPI2->DR);
//    DMA1_Stream4->M0AR  = (uint32_t)(adxrs290_regs /*+ ars2_dma_status*2*/);     
//    DMA1_Stream4->NDTR  = 2;
//    DMA1_Stream4->CR    |= DMA_SxCR_MINC | DMA_SxCR_PSIZE_0 | DMA_SxCR_MSIZE_0 | 
//                                DMA_SxCR_TCIE | DMA_SxCR_DIR_0 | DMA_SxCR_PL | DMA_SxCR_EN;     
//}

