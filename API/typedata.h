
#ifndef TYPEDATA_H_
#define TYPEDATA_H_

/*! \struct TYPES
 *          Device type data
 */

typedef   struct // if data type changes - remember to change "cInputTypeDataInit" !
{
  SBYTE     Name[TYPE_NAME_LENGTH + 1]; //!< Device name
  DATA8     Type;                       //!< Device type
  DATA8     Connection;
  DATA8     Mode;                       //!< Device mode
  DATA8     DataSets;
  DATA8     Format;
  DATA8     Figures;
  DATA8     Decimals;
  DATA8     Views;
  DATAF     RawMin;                     //!< Raw minimum value      (e.c. 0.0)
  DATAF     RawMax;                     //!< Raw maximum value      (e.c. 1023.0)
  DATAF     PctMin;                     //!< Percent minimum value  (e.c. -100.0)
  DATAF     PctMax;                     //!< Percent maximum value  (e.c. 100.0)
  DATAF     SiMin;                      //!< SI unit minimum value  (e.c. -100.0)
  DATAF     SiMax;                      //!< SI unit maximum value  (e.c. 100.0)
  UWORD     InvalidTime;                //!< mS from type change to valid data
  UWORD     IdValue;                    //!< Device id value        (e.c. 0 ~ UART)
  DATA8     Pins;                       //!< Device pin setup
  SBYTE     Symbol[SYMBOL_LENGTH + 1];  //!< SI unit symbol
  UWORD     Align;
}
TYPES;

#define   DATAF_NAN     ((float)0 / (float)0) //(0x7FC00000) from bytecodes.h l.1495

typedef   struct
{
  DATA8     Connection[INPUTS];
  DATA8     Type[INPUTS];
  DATA8     Mode[INPUTS];
}
DEVCON;


#endif //TYPEDATA_H_
