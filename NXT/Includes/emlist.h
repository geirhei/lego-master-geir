/**
 * Copyright (c) 2013, Ford Motor Company
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

    * Redistributions of source code must retain the above copyright notice,
      this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright notice,
      this list of conditions and the following disclaimer in the documentation
      and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef _EMLIST_H_
#define _EMLIST_H_

#include <stdbool.h>


struct LinkedListElement {
   void* value;
   struct LinkedListElement* next;
};
typedef struct LinkedListElement LinkedListElement;

/* Public: A singly linked list of void pointers.
 */
struct LinkedList {
   LinkedListElement* head;
};
typedef struct LinkedList LinkedList;

struct LinkedListIterator {
   LinkedListElement* next;
};
typedef struct LinkedListIterator LinkedListIterator;

/* Public: Allocate and initialize a new, empty LinkedList.
 *
 * This allocates memory for the list, so make sure to call
 * emlist_destroy(LinkedList*) when you're done.
 *
 * Returns a pointer to a new LinkedList.
 */
LinkedList* emlist_create(void);

/* Public: De-allocate memory for a list allocated on the heap.
 *
 * Be aware that if the list is not empty, this will leak memory - you must
 * remove and de-allocate the values in the list yourself if they were not on
 * the stack.
 */
void emlist_destroy(LinkedList* list);

/* Public: Initialize a LinkedList that has already been allocated, either
 * in static or dynamic memory.
 *
 * If the list has already been initialized and has elements, this will leak
 * memory.
 *
 * list - the list to initialize.
 */
void emlist_initialize(LinkedList* list);

/* Public: Free any memory associated with the list.
 *
 * Be aware that if the list is not empty, this will leak memory - you must
 * remove and de-allocate the values in the list yourself if they were not on
 * the stack.
 *
 * list - the list to de-initialize, created on the stack or heap.
 */
void emlist_deinitialize(LinkedList* list);

/* Public: Check if the list contains a value.
 *
 * This function is O(n).
 *
 * Returns true if the value is in the list.
 */
bool emlist_contains(LinkedList* list, void* value);

/* Public: Insert a value into the list.
 *
 * This function is O(n).
 * TODO prepend so this can be O(1).
 *
 * Returns true if the value was inserted successfully. Returns false if memory
 * could not be allocated for the new element.
 */
bool emlist_insert(LinkedList* list, void* value);

/* Public: Remove a value from the list, if it is in the list.
 *
 * This does not free any memory associated with the value, so remember to free
 * it yourself after removing from the list if necessary - you will leak memory
 * otherwise.
 *
 * This function is O(n).
 *
 * Returns true if the value was found in the list and succesfully removed.
 * Returns false if the value was not in the list.
 */
bool emlist_remove(LinkedList* list, void* value);

/* Public: Remove and return the first value in the list.
 *
 * This does not free any memory associated with the value, so remember to free
 * it yourself after removing from the list if necessary - you will leak memory
 * otherwise.
 *
 * This function is O(1).
 *
 * Returns the removed value if the list was not empty, otherwise NULL.
 */
void* emlist_pop(LinkedList* list);

/* Public: Determine the total size of the list.
 *
 * This function is O(n).
 *
 * Returns the number of elements in the list.
 */
int emlist_size(LinkedList* list);

/* Public: Check if the list is empty.
 *
 * This function is O(1).
 *
 * Returns true if the list is empty.
 */
bool emlist_is_empty(LinkedList* list);

LinkedListIterator emlist_iterator(LinkedList* list);

LinkedListElement*  emlist_iterator_next(LinkedListIterator* iterator);


#endif // _EMLIST_H_
