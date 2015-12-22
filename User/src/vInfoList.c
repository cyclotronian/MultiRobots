#include "vInfoList.h"
#include "vDemoTask.h"

pInfoNode robotListInit(){
  InfoNode *L = (InfoNode *)malloc(sizeof(InfoNode));
  if(L == NULL)
	halt(100);
  L->next = NULL;
  return L;
}

/*
typedef struct InfoNode
{
   struct rbNode rbInfo;
   struct InfoNode *next;
}InfoNode,*pInfoNode;    //24Bytes
*/

#if 0
void robotListCreatT(InfoNode **L){
  InfoNode *s,*r; int i = 0;
  *L = (InfoNode *)malloc(sizeof(InfoNode));
  r = *L;
  while(1){
	s = (InfoNode *)malloc(sizeof(InfoNode));  
	s->rbInfo = recBoardCastInfo();         //接受周围节点的广播消息,check it!		
	r->next = s;
	r = s;
	if(i++ == ROBOTS)
	  break;
  }  
  r->next = NULL;
}
#endif

void robotListDisplay(InfoNode *L){
	InfoNode *r = L->next->next;
	while(r != NULL)
	{
		//printf("Node=%d, X=%f,Y=%f,angel=%f\n",r->rbInfo.nodeID,r->rbInfo.rpos.locationX,r->rbInfo.rpos.locationY,r->rbInfo.rNorthAngel);
		r = r->next;
	}
	return;
}


//insert an element into the linkedlist
pInfoNode robotListInsert(pInfoNode L,int i,rbNode rbInfo){
  InfoNode *pre,*p;
  pre = L;
  int tempi = 0;
  for(tempi = 1; tempi < i;tempi++)
	pre = pre->next;  
  p = (InfoNode *)malloc(sizeof(InfoNode));
  p->rbInfo = rbInfo;            //maybe must use 2 poninter
  p->next = pre->next;
  pre->next = p;
  return L;
}

//delete linkedlist
pInfoNode robotListDelete(pInfoNode L,rbNode rbInfo){
  InfoNode *p,*pre;
  p = L->next;
  while(p->rbInfo.nodeID != rbInfo.nodeID){
	pre = p;
	p = p->next;
  }
  
  pre->next = p->next;
  free(p);
  return L;
}

void robotListDestroy(pInfoNode L){
  pInfoNode p = L, q = p->next;
  while(q!=NULL){
	free(p);
	p = q;
	q = p->next;
  }
  free(p);
}

InfoNode getDgNodeFromList(InfoNode *p,robotPos rb,int *flag){
	InfoNode *q;
	robotPos temp;
	float relDis=0;
	q = p;
	while((q->next != NULL)){	
	  	temp = q->rbInfo.rpos;
		relDis = getDistance(temp,rb);
		printf("relDis=%f\n",relDis);
		if(relDis <= 1)
		{
		  *flag = 1;
		  //printf("Node=%d\n",q->rbInfo.nodeID);
		  return *q;
		}
		else 
		  *flag = 0;
		  q = q->next;
	}
     *flag = 0;
	return *q;
}

#if 0
float getRelAngelFromList(InfoNode *rb){
	return (rb->rbInfo.rNorthAngel);
}

robotPos getRobotPos(InfoNode *rb){
	return rb->rbInfo.rpos;
}

#endif