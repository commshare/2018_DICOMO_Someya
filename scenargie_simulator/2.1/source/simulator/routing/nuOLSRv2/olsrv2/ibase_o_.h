//
// This file is automatically generated from tempalte/ibase.h
//

namespace NuOLSRv2Port { //ScenSim-Port://

/************************************************************//**
 * @addtogroup ibase_o
 * @{
 */

/** Current ibase_o. */
#define IBASE_O    (&OLSR->ibase_o)

////////////////////////////////////////////////////////////////
//
// Iterator
//

/** Gets iterator.
 *
 * @return iterator which points the first tuple
 */
#define  ibase_o_iter() \
    ((tuple_o_t*)(tuple_o_t*)IBASE_O->next)

/** Checks whether iterator points end of the ibase.
 *
 * @param iter
 * @return true if iter points to the end of ibase
 */
#define ibase_o_iter_is_end(iter) \
    ((nu_bool_t)((void*)(iter) == (void*)IBASE_O))

/** Gets next iterator.
 *
 * @param iter
 * @return iterator which points the next tuple
 */
#define ibase_o_iter_next(iter) \
    ((tuple_o_t*)(tuple_o_t*)(iter)->next)

/** Gets the pointer which points end of the ibase.
 *
 * @return iterator which points the end of the ibase
 */
#define ibase_o_iter_end()    ((tuple_o_t*)IBASE_O)

/** Traverses ibase.
 *
 * @param p
 */

/** Traverses the ibase.
 *
 * @param p
 */
#define FOREACH_O(p)                    \
    for (tuple_o_t* p = ibase_o_iter(); \
         !ibase_o_iter_is_end(p);       \
         p = ibase_o_iter_next(p))

/** Sets timeout to the tuple.
 *
 * @param tuple
 * @param t
 */
#define tuple_o_set_timeout(tuple, t)              \
    tuple_time_set_timeout((tuple_time_t*)(tuple), \
        &(OLSR)->ibase_o_time_list, (t))

////////////////////////////////////////////////////////////////
//
// Information Base
//

/** Clears the change flag.
 */
#define ibase_o_clear_change_flag() \
    do { IBASE_O->change = false; } \
    while (0)

/** Checks whether the ibase has been changed.
 */
#define ibase_o_is_changed() \
    ((nu_bool_t)(IBASE_O->change))

/** Sets the change flag of the ibase.
 */
#define ibase_o_change()           \
    do { IBASE_O->change = true; } \
    while (0)

/** Checks whether the ibase is empty.
 *
 * @return true if the ibase is empty
 */
#define ibase_o_is_empty()    ((void*)(IBASE_O) == (void*)IBASE_O->next)

/** Gets the size of the ibase.
 *
 * @return the size of the ibase
 */
#define ibase_o_size()        ((IBASE_O)->n)

/** Gets the top tuple of the ibase.
 *
 * @return the top tuple of the ibase
 */
#define ibase_o_head()        ((ibase_o_is_empty()) ? NULL : (IBASE_O)->next)

/** @} */

}//namespace// //ScenSim-Port://