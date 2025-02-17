#ifndef IN_GAME_TRAINING_MENUS_H
#define IN_GAME_TRAINING_MENUS_H
#include <ultra64.h>
#include "data.h"
#include "types.h"

char *frMenuTextFailReason(struct menuitem *item);
char *frMenuTextDifficultyName(struct menuitem *item);
char *frMenuTextTimeTakenValue(struct menuitem *item);
char *frMenuTextScoreValue(struct menuitem *item);
char *frMenuTextWeaponName(struct menuitem *item);
char *frMenuTextTargetsDestroyedValue(struct menuitem *item);
char *frMenuTextAccuracyValue(struct menuitem *item);
char *frMenuTextGoalScoreLabel(struct menuitem *item);
char *frMenuTextGoalScoreValue(struct menuitem *item);
char *frMenuTextMinAccuracyOrTargetsLabel(struct menuitem *item);
char *frMenuTextMinAccuracyOrTargetsValue(struct menuitem *item);
char *frMenuTextTimeLimitLabel(struct menuitem *item);
char *frMenuTextTimeLimitValue(struct menuitem *item);
char *frMenuTextAmmoLimitLabel(struct menuitem *item);
char *frMenuTextAmmoLimitValue(struct menuitem *item);
char *ciMenuTextChrBioName(struct menuitem *item);
char *ciMenuTextChrBioAge(struct menuitem *item);
char *ciMenuTextChrBioRace(struct menuitem *item);
char *ciMenuTextMiscBioName(struct menuitem *item);
char *dtMenuTextName(struct menuitem *item);
char *dtMenuTextOkOrResume(struct menuitem *item);
char *dtMenuTextCancelOrAbort(struct menuitem *item);
char *dtMenuTextTimeTakenValue(struct menuitem *item);
char *htMenuTextName(struct menuitem *item);
char *htMenuTextOkOrResume(struct menuitem *item);
char *htMenuTextCancelOrAbort(struct menuitem *item);
char *htMenuTextTimeTakenValue(struct menuitem *item);
char *bioMenuTextName(struct menuitem *item);
char *ciMenuTextHangarBioSubheading(struct menuitem *item);
struct menudialogdef *ciGetFrWeaponListMenuDialog(void);
MenuDialogHandlerResult frTrainingInfoMenuDialog(s32 operation, struct menudialogdef *dialogdef, union handlerdata *data);
MenuDialogHandlerResult frTrainingStatsMenuDialog(s32 operation, struct menudialogdef *dialogdef, union handlerdata *data);
MenuDialogHandlerResult ciCharacterProfileMenuDialog(s32 operation, struct menudialogdef *dialogdef, union handlerdata *data);
MenuDialogHandlerResult dtTrainingDetailsMenuDialog(s32 operation, struct menudialogdef *dialogdef, union handlerdata *data);
MenuDialogHandlerResult menudialogDeviceTrainingResults(s32 operation, struct menudialogdef *dialogdef, union handlerdata *data);
MenuDialogHandlerResult menudialog001a6aa4(s32 operation, struct menudialogdef *dialogdef, union handlerdata *data);
MenuDialogHandlerResult menudialogFiringRangeResults(s32 operation, struct menudialogdef *dialogdef, union handlerdata *data);
MenuDialogHandlerResult ciHangarHolographMenuDialog(s32 operation, struct menudialogdef *dialogdef, union handlerdata *data);
MenuItemHandlerResult frDetailsOkMenuHandler(s32 operation, struct menuitem *item, union handlerdata *data);
MenuItemHandlerResult frAbortMenuHandler(s32 operation, struct menuitem *item, union handlerdata *data);
MenuItemHandlerResult frWeaponListMenuHandler(s32 operation, struct menuitem *item, union handlerdata *data);
MenuItemHandlerResult frScoringMenuHandler(s32 operation, struct menuitem *item, union handlerdata *data);
MenuItemHandlerResult menuhandlerFrFailedContinue(s32 operation, struct menuitem *item, union handlerdata *data);
MenuItemHandlerResult ciOfficeInformationMenuHandler(s32 operation, struct menuitem *item, union handlerdata *data);
MenuItemHandlerResult dtDeviceListMenuHandler(s32 operation, struct menuitem *item, union handlerdata *data);
MenuItemHandlerResult menuhandlerDtOkOrResume(s32 operation, struct menuitem *item, union handlerdata *data);
MenuItemHandlerResult menuhandler001a6514(s32 operation, struct menuitem *item, union handlerdata *data);
MenuItemHandlerResult htHoloListMenuHandler(s32 operation, struct menuitem *item, union handlerdata *data);
MenuItemHandlerResult menuhandler001a6a34(s32 operation, struct menuitem *item, union handlerdata *data);
MenuItemHandlerResult menuhandler001a6a70(s32 operation, struct menuitem *item, union handlerdata *data);
MenuItemHandlerResult ciHangarInformationMenuHandler(s32 operation, struct menuitem *item, union handlerdata *data);
MenuItemHandlerResult ciHangarTitleMenuHandler(s32 operation, struct menuitem *item, union handlerdata *data);
MenuItemHandlerResult frDifficultyMenuHandler(s32 operation, struct menuitem *item, union handlerdata *data);

#endif
